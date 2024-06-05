from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    
    # Nodo de proyecto
    nav_project_node = Node(
        package='nav_project', executable='nav_project_ex', output='screen')

    supervisor_node = Node(
        package='nav_project',  # Reemplaza con el nombre de tu paquete
        executable='supervisor_ex',
        name='supervisor_node',
        output='screen'
    )
    

    # Crear y retornar la descripción del lanzamiento
    return LaunchDescription([
        nav_project_node,
        supervisor_node
    ])              