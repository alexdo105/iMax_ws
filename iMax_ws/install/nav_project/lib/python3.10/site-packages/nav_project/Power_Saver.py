#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import DockStatus
from std_srvs.srv import Empty
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PowerSaverNode(Node):
    def __init__(self):
        super().__init__('power_saver_node')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Or whatever queue size you prefer
        )
        self.subscription = self.create_subscription(
            DockStatus,
            'dock_status',
            self.dock_status_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

        # Clients to stop/start RPLIDAR and OAK-D
        self.rplidar_stop_client = self.create_client(Empty, 'stop_motor')
        self.oakd_stop_client = self.create_client(Empty, 'oakd/stop_camera')

        # Ensure that services are available
        #while not self.rplidar_stop_client.wait_for_service(timeout_sec=1.0):
            #self.get_logger().info('RPLIDAR stop service not available, waiting again...')
        #while not self.oakd_stop_client.wait_for_service(timeout_sec=1.0):
            #self.get_logger().info('OAK-D stop service not available, waiting again...')

    def dock_status_callback(self, msg):
        if msg.is_docked:
            self.get_logger().info('Docked: Activating power saver mode...')
            self.activate_power_saver_mode()
        else:
            self.get_logger().info('Undocked: Deactivating power saver mode...')

    def activate_power_saver_mode(self):
        # Asegúrate de que el cliente espera a que el servicio esté disponible
        if not self.rplidar_stop_client.service_is_ready():
            self.rplidar_stop_client.wait_for_service()
            self.get_logger().info('El servicio RPLIDAR stop ahora está disponible.')

        
        # Stop RPLIDAR
        req = Empty.Request()
        future = self.rplidar_stop_client.call_async(req)
        future.add_done_callback(self.rplidar_stop_response_callback)

        # Stop OAK-D
        req = Empty.Request()
        future = self.oakd_stop_client.call_async(req)
        future.add_done_callback(self.oakd_stop_response_callback)

    def rplidar_stop_response_callback(self, future):
        try:
            future.result()
            self.get_logger().info('RPLIDAR detenido con éxito.')
        except Exception as e:
            self.get_logger().error('La llamada al servicio falló %r' % (e,))

    def oakd_stop_response_callback(self, future):
        try:
            future.result()
            self.get_logger().info('OAK-D successfully stopped.')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))
def main(args=None):
    rclpy.init(args=args)
    power_saver_node = PowerSaverNode()
    rclpy.spin(power_saver_node)
    power_saver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
