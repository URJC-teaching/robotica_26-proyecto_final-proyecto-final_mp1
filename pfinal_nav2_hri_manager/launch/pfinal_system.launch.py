from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    Genera el grafo de ejecucion del sistema completo.
    Coordina Kobuki (Real/Sim), Nav2, HRI, YOLO 3D y el Manager.
    """
    kobuki_share = FindPackageShare('kobuki')
    hri_share = FindPackageShare('simple_hri')
    yolo_share = FindPackageShare('yolo_bringup')
    manager_share = FindPackageShare('pfinal_nav2_hri_manager')

    # DEFINICION DE ARGUMENTOS DE LANZAMIENTO
    declared_arguments = [
        # mode: Determina si usamos el simulador ('sim') o el robot real ('real')
        DeclareLaunchArgument('mode', default_value='sim'),
        # Flags para activar/desactivar componentes especificos
        DeclareLaunchArgument('start_kobuki', default_value='false'), # Cargar el robot/Gazebo
        DeclareLaunchArgument('start_nav2', default_value='true'),   # Cargar navegacion
        DeclareLaunchArgument('start_hri', default_value='true'),    # Cargar voz
        DeclareLaunchArgument('start_yolo', default_value='true'),   # Cargar vision
        DeclareLaunchArgument('start_manager', default_value='true'),# Cargar coordinador de mision
        
        # hri_stack: Tipo de motor de voz ('free' -> gratuito, 'local' -> sin internet, 'cloud' -> google)
        DeclareLaunchArgument('hri_stack', default_value='free'),
        
        # world/map: Rutas a los archivos de entorno
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare('aws_robomaker_small_house_world'),
                    'worlds',
                    'small_house.world',
                ]
            ),
        ),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([kobuki_share, 'maps', 'aws_house.yaml']),
        ),
        
        # Parametros de Nav2 segun el modo (muy importante para evitar choques)
        DeclareLaunchArgument(
            'nav2_params_sim',
            default_value=PathJoinSubstitution(
                [kobuki_share, 'config', 'kobuki_sim_nav_params.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'nav2_params_real',
            default_value=PathJoinSubstitution([kobuki_share, 'config', 'kobuki_nav_params.yaml']),
        ),
        
        # Configuracion de sensores (solo usados en modo 'real')
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('xtion', default_value='false'),
        DeclareLaunchArgument('astra', default_value='true'),
        DeclareLaunchArgument('lidar_a2', default_value='true'),
        DeclareLaunchArgument('lidar_s2', default_value='false'),
        DeclareLaunchArgument('namespace', default_value=''),
        
        # Yolo y Orquestador
        DeclareLaunchArgument('model', default_value='yolov8n.pt'),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('threshold', default_value='0.5'),
        DeclareLaunchArgument('input_image_topic', default_value='/camera/rgb/image_raw'),
        DeclareLaunchArgument('input_depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('input_depth_info_topic', default_value='/camera/depth/camera_info'),
        DeclareLaunchArgument('yolo_namespace', default_value='yolo'),
        DeclareLaunchArgument('detection_topic', default_value='/yolo/detections_3d'),
        DeclareLaunchArgument(
            'orchestrator_params',
            default_value=PathJoinSubstitution([manager_share, 'config', 'mission_params.yaml']),
        ),
        DeclareLaunchArgument(
            'named_goals_file',
            default_value=PathJoinSubstitution([manager_share, 'config', 'named_goals.yaml']),
        ),
    ]

    # --- COMPONENTE 1: ROBOT EN SIMULACION ---
    sim_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([kobuki_share, 'launch', 'simulation.launch.py'])
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration('mode'),
                    "' == 'sim' and '",
                    LaunchConfiguration('start_kobuki'),
                    "'.lower() == 'true'",
                ]
            )
        ),
    )

    sim_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([kobuki_share, 'launch', 'navigation_sim.launch.py'])
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('nav2_params_sim'),
            'rviz': LaunchConfiguration('rviz'),
            'use_sim_time': 'true',
        }.items(),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration('mode'),
                    "' == 'sim' and '",
                    LaunchConfiguration('start_nav2'),
                    "'.lower() == 'true'",
                ]
            )
        ),
    )

    real_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([kobuki_share, 'launch', 'kobuki.launch.py'])
        ),
        launch_arguments={
            'xtion': LaunchConfiguration('xtion'),
            'astra': LaunchConfiguration('astra'),
            'lidar_a2': LaunchConfiguration('lidar_a2'),
            'lidar_s2': LaunchConfiguration('lidar_s2'),
            'namespace': LaunchConfiguration('namespace'),
        }.items(),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration('mode'),
                    "' == 'real' and '",
                    LaunchConfiguration('start_kobuki'),
                    "'.lower() == 'true'",
                ]
            )
        ),
    )

    real_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([kobuki_share, 'launch', 'navigation.launch.py'])
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('nav2_params_real'),
            'rviz': LaunchConfiguration('rviz'),
            'use_sim_time': 'false',
            'namespace': LaunchConfiguration('namespace'),
        }.items(),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration('mode'),
                    "' == 'real' and '",
                    LaunchConfiguration('start_nav2'),
                    "'.lower() == 'true'",
                ]
            )
        ),
    )

    hri_free = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([hri_share, 'launch', 'free_simple_hri.launch.py'])
        ),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration('start_hri'),
                    "'.lower() == 'true' and '",
                    LaunchConfiguration('hri_stack'),
                    "' == 'free'",
                ]
            )
        ),
    )

    hri_local = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([hri_share, 'launch', 'local_simple_hri.launch.py'])
        ),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration('start_hri'),
                    "'.lower() == 'true' and '",
                    LaunchConfiguration('hri_stack'),
                    "' == 'local'",
                ]
            )
        ),
    )

    hri_cloud = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([hri_share, 'launch', 'simple_hri.launch.py'])
        ),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration('start_hri'),
                    "'.lower() == 'true' and '",
                    LaunchConfiguration('hri_stack'),
                    "' == 'cloud'",
                ]
            )
        ),
    )

    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([yolo_share, 'launch', 'yolo.launch.py'])
        ),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'device': LaunchConfiguration('device'),
            'threshold': LaunchConfiguration('threshold'),
            'use_tracking': 'True',
            'use_3d': 'True',
            'input_image_topic': LaunchConfiguration('input_image_topic'),
            'input_depth_topic': LaunchConfiguration('input_depth_topic'),
            'input_depth_info_topic': LaunchConfiguration('input_depth_info_topic'),
            'namespace': LaunchConfiguration('yolo_namespace'),
            'target_frame': 'base_link',
            'use_debug': 'True',
        }.items(),
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration('start_yolo'), "'.lower() == 'true'"]
            )
        ),
    )

    # Evaluamos explicitamente a 'true' o 'false' para rclpy
    use_sim_time_param = PythonExpression([
        "'true' if '", LaunchConfiguration('mode'), "' == 'sim' else 'false'"
    ])

    manager_node = Node(
        package='pfinal_nav2_hri_manager',
        executable='mission_coordinator',
        name='mission_coordinator',
        output='screen',
        parameters=[
            LaunchConfiguration('orchestrator_params'),
            {
                'named_goals_file': LaunchConfiguration('named_goals_file'),
                'detection_topic': LaunchConfiguration('detection_topic'),
                'use_sim_time': use_sim_time_param
            },
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('start_manager'), "'.lower() == 'true'"])
        ),
    )

    return LaunchDescription(
        declared_arguments
        + [
            sim_robot,
            sim_nav2,
            real_robot,
            real_nav2,
            hri_free,
            hri_local,
            hri_cloud,
            yolo,
            manager_node,
        ]
    )

# ==============================================================================
# GUIA DE EJECUCION MANUAL (POR PIEZAS)
# ==============================================================================
# Si algo falla o quieres ejecutar los componentes por separado, usa:
#
# 1. NAVEGACION (Nav2):
#    ros2 launch kobuki navigation_sim.launch.py map:=/ruta/mapa.yaml use_sim_time:=true
#
# 2. VOZ (HRI):
#    ros2 launch simple_hri free_simple_hri.launch.py
#
# 3. VISION (YOLO 3D):
#    ros2 launch yolo_bringup yolo.launch.py use_3d:=True target_frame:=base_link
#
# 4. COORDINADOR (Este paquete):
#    ros2 run pfinal_nav2_hri_manager mission_coordinator --ros-args \
#         -p named_goals_file:=/ruta/pfinal_nav2_hri_manager/config/named_goals.yaml
# ==============================================================================
