from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# Declarar todos los argumentos al inicio
ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time', default_value='false', choices=['true', 'false'],
        description='Use simulation time'),
    
    DeclareLaunchArgument(
        'namespace', default_value='', description='Robot namespace'),
    
    DeclareLaunchArgument(
        'params_file', default_value=PathJoinSubstitution([
            get_package_share_directory('turtlebot4_navigation'), 'config', 'nav2.yaml']),
        description='Nav2 parameters'),
    
    DeclareLaunchArgument(
        'params', default_value=PathJoinSubstitution([
            get_package_share_directory('turtlebot4_navigation'), 'config', 'localization.yaml']),
        description='Localization parameters'),

    DeclareLaunchArgument(
        'map', default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'), 'CIMA16102NUEVO.yaml']),
        description='Full path to the map yaml file to load')
]

def generate_launch_description():
    # Directorios de paquetes
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_turtlebot4_navigation = get_package_share_directory('turtlebot4_navigation')

    # Configuraciones de lanzamiento
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Nodo de proyecto
    nav_project_node = Node(
        package='nav_project', executable='nav_project_ex', output='screen',
        namespace=namespace)

    # Inclusión de la descripción de lanzamiento para localización
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [pkg_nav2_bringup, 'launch', 'localization_launch.py'])),
        launch_arguments={
            'namespace': namespace,
            'map': LaunchConfiguration('map'),
            'use_sim_time': use_sim_time,
            'params_file': LaunchConfiguration('params')}.items())

    # Inclusión de nav2 con configuraciones correctas   
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': use_sim_time,
            'namespace': namespace}.items())

    # Crear y retornar la descripción del lanzamiento
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(nav2)
    ld.add_action(localization)
    ld.add_action(nav_project_node)
    
    return ld
