# launch/pfinal_nav2_hri_manager.launch.py
#
# Launch principal del proyecto:
#   - Nav2 + mapa (kobuki/navigation_sim.launch.py)     [include_nav2:=true,  solo en sim]
#   - HRI stack local                                    [include_hri:=true]
#   - Pipeline YOLO 3D                                   [use_yolo:=true]
#   - Mission manager (FSM Nav2 + HRI + YOLO)            [run_mission:=true]
#   - Nodos de prueba HRI / Nav2                         [test_hri / test_nav]
#
# Modos:
#   ros2 launch pfinal_nav2_hri_manager pfinal_nav2_hri_manager.launch.py mode:=sim
#   ros2 launch pfinal_nav2_hri_manager pfinal_nav2_hri_manager.launch.py            # mode:=real

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    use_sim_time = (mode == 'sim')

    # ---- Topics y frames según modo (igual estructura que full_vff_3d.launch.py) ----
    if mode == 'sim':
        image_topic  = '/rgbd_camera/image'
        depth_topic  = '/rgbd_camera/depth_image'
        info_topic   = '/rgbd_camera/camera_info'
        camera_frame = 'camera_link'
        base_frame   = 'base_link'
    else:
        image_topic  = '/camera/rgb/image_raw'
        depth_topic  = '/camera/depth_raw/image'
        info_topic   = '/camera/depth_raw/camera_info'
        camera_frame = 'camera_rgb_frame'
        base_frame   = 'base_footprint'

    pkg_share = get_package_share_directory('pfinal_nav2_hri_manager')
    waypoints_file = os.path.join(pkg_share, 'config', 'waypoints.yaml')

    # Mapa editado en la raíz del workspace (coordenadas alineadas con waypoints.yaml)
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    map_edited = os.path.join(ws_root, 'map_edited.yaml')

    include_nav2 = LaunchConfiguration('include_nav2').perform(context).lower() == 'true'
    include_hri  = LaunchConfiguration('include_hri').perform(context).lower() == 'true'
    use_yolo     = LaunchConfiguration('use_yolo').perform(context).lower() == 'true'

    actions = []

    # ---- Nav2 (solo si está disponible el bringup de kobuki) ----
    if include_nav2:
        try:
            kobuki_share = get_package_share_directory('kobuki')
            nav_launch = (
                'navigation_sim.launch.py' if mode == 'sim' else 'navigation.launch.py'
            )
            nav_launch_path = os.path.join(kobuki_share, 'launch', nav_launch)
            if os.path.isfile(nav_launch_path):
                nav_args = {
                    'use_sim_time': str(use_sim_time).lower(),
                    'rviz':         'False',
                }
                if os.path.isfile(map_edited):
                    nav_args['map'] = map_edited
                actions.append(IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav_launch_path),
                    launch_arguments=nav_args.items(),
                ))
        except PackageNotFoundError:
            pass  # Si no hay paquete kobuki, hay que lanzar Nav2 a mano aparte

    # ---- HRI stack (stt_service_local + tts_service_local + extract + yesno + sound_play) ----
    if include_hri:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'hri_bringup.launch.py')
            ),
        ))

    # ---- YOLO bringup (modelo nano, CPU) ----
    if use_yolo:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('yolo_bringup'),
                    'launch', 'yolo.launch.py',
                )
            ),
            launch_arguments={
                'input_image_topic':      image_topic,
                'input_depth_topic':      depth_topic,
                'input_depth_info_topic': info_topic,
                'target_frame':           camera_frame,
                'model':                  'yolov8n.pt',
                'device':                 'cpu',
                'use_sim_time':           str(use_sim_time).lower(),
            }.items(),
        ))

        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('camera'),
                    'launch', 'yolo_detection3d.launch.py',
                )
            ),
            launch_arguments={'use_sim_time': str(use_sim_time).lower()}.items(),
        ))

    # ---- Mission manager ----
    actions.append(Node(
        package='pfinal_nav2_hri_manager',
        executable='mission_manager',
        name='mission_manager_node',
        output='screen',
        parameters=[{
            'use_sim_time':         use_sim_time,
            'waypoints_file':       waypoints_file,
            'detections_topic':     '/detections_3d',
            'person_class':         'person',
            'person_hold_distance': 1.5,
            'person_hold_time':     2.0,
            'goal_timeout_sec':     90.0,
            'post_goal_wait_sec':   2.0,
            'base_frame':           base_frame,
            'map_frame':            'map',
            'random_xmin':           0.05,
            'random_xmax':           1.45,
            'random_ymin':           2.6,
            'random_ymax':           5.5,
        }],
        condition=IfCondition(LaunchConfiguration('run_mission')),
    ))

    # ---- Nodos de prueba opcionales ----
    actions.append(Node(
        package='pfinal_nav2_hri_manager',
        executable='hri_test',
        name='hri_test_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('test_hri')),
    ))
    actions.append(Node(
        package='pfinal_nav2_hri_manager',
        executable='nav_test',
        name='nav_test_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'goal_x':       0.5,
            'goal_y':       0.0,
            'goal_theta':   0.0,
        }],
        condition=IfCondition(LaunchConfiguration('test_nav')),
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='real',
            description='real | sim — selecciona topics y frames automáticamente'),
        DeclareLaunchArgument(
            'include_nav2', default_value='false',
            description='Lanzar Nav2 + mapa (kobuki/navigation_sim.launch.py)'),
        DeclareLaunchArgument(
            'include_hri', default_value='true',
            description='Lanzar el stack HRI local (STT + TTS + extract + yesno)'),
        DeclareLaunchArgument(
            'use_yolo', default_value='true',
            description='Lanzar el pipeline YOLO 3D'),
        DeclareLaunchArgument(
            'run_mission', default_value='true',
            description='Lanzar el mission manager (FSM)'),
        DeclareLaunchArgument(
            'test_hri', default_value='false',
            description='Lanzar el nodo de prueba HRI (necesita run_mission:=false)'),
        DeclareLaunchArgument(
            'test_nav', default_value='false',
            description='Lanzar el nodo de prueba Nav2 (necesita run_mission:=false)'),
        OpaqueFunction(function=launch_setup),
    ])
