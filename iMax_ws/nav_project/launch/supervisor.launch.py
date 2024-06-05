from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Nodo supervisor
    supervisor_node = Node(
        package='nav_project',  # Reemplaza con el nombre de tu paquete
        executable='supervisor_ex',
        name='supervisor_node',
        output='screen'
    )

    # Crear y retornar la descripción del lanzamiento
    return LaunchDescription([
        supervisor_node
    ])
