# launch/yolo_stack.launch.py
#
# Pila YOLO ligera (2D, sin detect_3d_node) + traductor de mensajes.
# Funciona en VM/CPU sin imagen de profundidad.
#
# Pipeline:
#   yolo_bringup (yolo_node + debug_node, use_3d:=False)
#       /yolo/detections  (yolo_msgs/DetectionArray, 2D)
#       /yolo/dbg_image   (imagen anotada para debug)
#   yolo_depth_node  — estima distancia con altura del bbox + fy
#       /detections_3d  (vision_msgs/Detection3DArray) ← mission_manager
#
# Uso (dos terminales, en este orden):
#   T1) ros2 launch pfinal_nav2_hri_manager yolo_stack.launch.py mode:=sim
#       # esperar a ver: [yolo_node]: Model loaded  Y  [yolo_depth_node]: CameraInfo recibida
#       # verificar:  ros2 topic hz /detections_3d
#       # debug visual: ros2 run rqt_image_view rqt_image_view /yolo/dbg_image
#
#   T2) ros2 launch pfinal_nav2_hri_manager pfinal_nav2_hri_manager.launch.py \
#               mode:=sim use_yolo:=false
#
# Equivalente manual del T1:
#   ros2 launch yolo_bringup yolo.launch.py \
#       input_image_topic:=/rgbd_camera/image model:=yolov8n.pt device:=cpu \
#       use_tracking:=False use_3d:=False use_debug:=True \
#       imgsz_height:=320 imgsz_width:=320 threshold:=0.4 \
#       image_reliability:=2 use_sim_time:=true
#   ros2 run pfinal_nav2_hri_manager yolo_depth --ros-args \
#       -p camera_frame:=camera_link -p use_sim_time:=true \
#       -r detections:=/yolo/detections \
#       -r camera_info:=/rgbd_camera/camera_info \
#       -r detections_3d:=/detections_3d

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    mode         = LaunchConfiguration('mode').perform(context)
    use_sim_time = (mode == 'sim')

    if mode == 'sim':
        image_topic  = '/rgbd_camera/image'
        info_topic   = '/rgbd_camera/camera_info'
        camera_frame = 'camera_link'
    else:
        image_topic  = '/camera/rgb/image_raw'
        info_topic   = '/camera/rgb/camera_info'
        camera_frame = 'camera_rgb_frame'

    pkg_share  = get_package_share_directory('pfinal_nav2_hri_manager')
    ws_root    = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    venv_site  = os.path.join(ws_root, 'venv_asr', 'lib', 'python3.12', 'site-packages')
    pythonpath = (
        (venv_site + ':' + os.environ.get('PYTHONPATH', '')).rstrip(':')
        if os.path.isdir(venv_site)
        else os.environ.get('PYTHONPATH', '')
    )

    use_debug = LaunchConfiguration('use_debug').perform(context)

    actions = [SetEnvironmentVariable('PYTHONPATH', pythonpath)]

    # ---- YOLO 2D (idéntico al comando que funciona) ----
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yolo_bringup'),
                'launch', 'yolo.launch.py',
            )
        ),
        launch_arguments={
            'input_image_topic': image_topic,
            'model':             'yolov8n.pt',
            'device':            'cpu',
            'use_tracking':      'False',
            'use_3d':            'False',       # sin detect_3d_node (pesa mucho en VM)
            'use_debug':         use_debug,
            'image_reliability': '2',           # BEST_EFFORT — igual que sim/openni2
            'imgsz_height':      '320',
            'imgsz_width':       '320',
            'threshold':         '0.4',
            'use_sim_time':      str(use_sim_time).lower(),
        }.items(),
    ))

    # ---- yolo_depth_node: yolo_msgs/DetectionArray → vision_msgs/Detection3DArray ----
    # Estima la distancia con: d = (person_height_m * fy) / bbox_height_px
    # No necesita imagen de profundidad, sólo camera_info de la cámara RGB.
    actions.append(Node(
        package='pfinal_nav2_hri_manager',
        executable='yolo_depth',
        name='yolo_depth_node',
        output='screen',
        parameters=[{
            'use_sim_time':       use_sim_time,
            'camera_frame':       camera_frame,
            'target_class':       'person',
            'person_height_m':    1.7,
            'min_bbox_height_px': 20,
            'max_distance_m':     6.0,
        }],
        remappings=[
            ('detections',    '/yolo/detections'),
            ('camera_info',   info_topic),
            ('detections_3d', '/detections_3d'),
        ],
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='sim',
            description='real | sim — selecciona topics y frames'),
        DeclareLaunchArgument(
            'use_debug', default_value='True',
            description='Activa debug_node de YOLO → /yolo/dbg_image'),
        OpaqueFunction(function=launch_setup),
    ])
