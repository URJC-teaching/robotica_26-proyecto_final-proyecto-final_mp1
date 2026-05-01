# launch/pfinal_nav2_hri_manager.launch.py
# Lanza el proyecto completo: misión + (opcionales) pruebas individuales

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('pfinal_nav2_hri_manager')
    waypoints_file = os.path.join(pkg_share, 'config', 'waypoints.yaml')

    # ---- Argumentos configurables desde CLI ----
    args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Usar tiempo de simulación (Gazebo/Webots)'
        ),
        DeclareLaunchArgument(
            'follow_topic',
            default_value='/yolo/target_pose',
            description='Topic donde YOLO publica la pose del objetivo'
        ),
        DeclareLaunchArgument(
            'post_goal_wait_sec',
            default_value='2.0',
            description='Segundos de espera tras completar un goal antes de pedir nuevo comando'
        ),
        DeclareLaunchArgument(
            'goal_timeout_sec',
            default_value='60.0',
            description='Timeout máximo por goal de navegación'
        ),
        # Modo de prueba: lanza solo HRI o solo Nav
        DeclareLaunchArgument(
            'test_hri',
            default_value='false',
            description='Lanza solo el nodo de prueba HRI'
        ),
        DeclareLaunchArgument(
            'test_nav',
            default_value='false',
            description='Lanza solo el nodo de prueba Nav2'
        ),
    ]

    use_sim_time    = LaunchConfiguration('use_sim_time')
    follow_topic    = LaunchConfiguration('follow_topic')
    post_goal_wait  = LaunchConfiguration('post_goal_wait_sec')
    goal_timeout    = LaunchConfiguration('goal_timeout_sec')
    test_hri        = LaunchConfiguration('test_hri')
    test_nav        = LaunchConfiguration('test_nav')

    # ---- Nodo principal (misión completa) ----
    mission_node = Node(
        package='pfinal_nav2_hri_manager',
        executable='mission_manager',
        name='mission_manager_node',
        output='screen',
        parameters=[{
            'use_sim_time':        use_sim_time,
            'waypoints_file':      waypoints_file,
            'follow_topic':        follow_topic,
            'post_goal_wait_sec':  post_goal_wait,
            'goal_timeout_sec':    goal_timeout,
        }],
        condition=IfCondition('true'),  # Siempre activo si no hay modo test
    )

    # ---- Nodo de prueba HRI (standalone) ----
    hri_test_node = Node(
        package='pfinal_nav2_hri_manager',
        executable='hri_test',
        name='hri_test_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(test_hri),
    )

    # ---- Nodo de prueba Nav2 (standalone) ----
    nav_test_node = Node(
        package='pfinal_nav2_hri_manager',
        executable='nav_test',
        name='nav_test_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'goal_x': 2.5,
            'goal_y': 1.0,
            'goal_theta': 0.0,
        }],
        condition=IfCondition(test_nav),
    )

    return LaunchDescription(args + [
        mission_node,
        hri_test_node,
        nav_test_node,
    ])
