#####Este codigo es el que funciona, se guardo el 23 mayo 14:26



import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from nav2_simple_commander.robot_navigator import TaskResult
from std_srvs.srv import Empty
from threading import Lock, Thread
from time import sleep
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import BatteryState
from irobot_create_msgs.msg import DockStatus
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from turtlebot4_msgs.msg import UserLed
import math
from geometry_msgs.msg import Twist
from time import sleep
class CompleteNode(Node):
    def __init__(self):
        super().__init__('CIMA16102_Test')
        self.navigator = TurtleBot4Navigator()
        self.iot_callback_group = ReentrantCallbackGroup()
        qos_profile_wifiarduino = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Confiabilidad asegurada
            durability=DurabilityPolicy.VOLATILE,    # Durabilidad cambiada a VOLATILE
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Profundidad de la historia
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.iot_subscription = self.create_subscription(
            Int32,
            'wifiarduino',
            self.iot_callback,
            qos_profile=qos_profile_wifiarduino,  # QoS profile
            callback_group=self.iot_callback_group
            
        )

        self.iot_subscription = self.create_subscription(
            Int32,
            'wifiarduino2',
            self.iot_callback_2,
            qos_profile=qos_profile_wifiarduino,  # QoS profile
            callback_group=self.iot_callback_group
            
        )
        
         # Define QoS profile with history and depth
        qos_profile_led = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create the publisher
        self.led_publisher = self.create_publisher(UserLed, '/hmi/led', qos_profile_led)


        self.battery_state_subscriber = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_state_callback,
            qos_profile_sensor_data)

        self.last_known_battery = None
        self.green_blink_led_right()
        # Initialize the robot
        self.start_on_dock()

        # Create a timer for battery status checks
        self.timer = self.create_timer(
            60,  # 1 minute
            self.check_battery_status,
            callback_group=self.iot_callback_group
        )

        self.set_initial_pose(0.115, 0.033, math.degrees(-0.100))
        self.get_logger().info('Initial pose is the new one')
        self.get_logger().info('Initial pose has been set to the docking area')


        # Wait for the nav2 to be active.
        self.get_logger().info('I guess Nav2 is active')
        self.get_logger().info('Waiting for instructions')
    #    self.topic_check_timer = self.create_timer(0.5, self.check_topic_availability)

        
    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_msg)
        self.get_logger().info('Published stop command to /cmd_vel')


    def turn_off_leds(self):
        self.turn_off_led_right()
        self.turn_off_led()
        self.get_logger().info("Turning off both leds")


    def turn_off_led(self):
        msg = UserLed()
        msg.led = 0  # LED ID
        msg.color = 0  # Assuming '0' is the color for 'off'
        msg.blink_period = 0  # No blink
        msg.duty_cycle = 0.0  # Fully off
        self.led_publisher.publish(msg)
        self.get_logger().info(f'Published to turn off LED: {msg}')

    def turn_off_led_right(self):
        msg = UserLed()
        msg.led = 1  # LED ID for the right LED
        msg.color = 0  # Assuming '0' is the color for 'off'
        msg.blink_period = 0  # No blink
        msg.duty_cycle = 0.0  # Fully off
        self.led_publisher.publish(msg)
        self.get_logger().info(f'Published to turn off right LED: {msg}')
        
    
    def green_blink_led_right(self):
        self.turn_off_leds()
        msg = UserLed()
        msg.led = 1  # LED ID for the right LED
        msg.color = 1  # Assuming '2' is the color for 'red'
        msg.blink_period = 1000  # 1 second blink period
        msg.duty_cycle = 0.5  # 50% duty cycle for blinking
        self.led_publisher.publish(msg)
        self.get_logger().info(f'Published to blink right LED in green: {msg}')
    



    def start_on_dock(self):
        if not self.navigator.getDockedStatus():
            self.navigator.info('Docking before initialising pose')
            self.navigator.dock()
            # Wait for docking to complete
            while not self.navigator.isDockComplete():
                rclpy.spin_once(self.navigator, timeout_sec=0.5)
            self.get_logger().info('Dock completed')
        elif self.navigator.getDockedStatus():
            self.get_logger().info('Already docked.')



    def set_initial_pose(self, x, y,z):
        initial_pose = self.navigator.getPoseStamped([x, y], z)
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info('Initial pose has been set, waiting for nav2 to be active')
        self.get_logger().info('Nav2 is active')




    def go_to_point_1(self):
        goal_pose = self.navigator.getPoseStamped([-1.655, -1.599], math.degrees(3.130))
        # Go to each goal pose
        self.navigator.startToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.navigator, timeout_sec=0.5)
        self.navigator.info('The Turtlebot has reached position 1')
        
    def go_to_point_2(self):
        goal_pose = self.navigator.getPoseStamped([-0.206, 1.318], math.degrees(3.116))

        # Go to each goal pose
        self.navigator.startToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.navigator, timeout_sec=0.5)
        self.navigator.info('The Turtlebot has reached position 2')

    def go_to_point_3(self):
        goal_pose = self.navigator.getPoseStamped([-0.918, -3.041], math.degrees(2.955))

        # Go to each goal pose
        self.navigator.startToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.navigator, timeout_sec=0.5)
        self.navigator.info('The Turtlebot has reached position 3')

    def go_to_point_5(self):
        goal_pose = self.navigator.getPoseStamped([-2.696, 1.354], math.degrees(3.097))

        # Go to each goal pose
        self.navigator.startToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.navigator, timeout_sec=0.5)
        self.navigator.info('The Turtlebot has reached position 3')

    def dock_again(self):
        goal_pose = self.navigator.getPoseStamped([-0.162, -0.034], math.degrees(-0.090))

        # Go to each goal pose
        self.navigator.startToPose(goal_pose)
        self.navigator.info('Going near the docking station')

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.navigator, timeout_sec=0.5)
        self.get_logger().info('Robot has reached the position to dock easily')
        self.navigator.info('Initializing docking process')
        self.navigator.dock()
        self.get_logger().info('Docking...')
        # Wait for docking to complete
        while not self.navigator.isDockComplete():
            rclpy.spin_once(self.navigator, timeout_sec=0.5) 


    def iot_callback(self, msg):
        self.get_logger().info(f'IoT Command received: "{msg.data}"')
        if self.navigator.getDockedStatus():
            self.get_logger().info("Robot undocking and starting the mission")
            self.get_logger().info("Checking if Nav2 still active")
            self.get_logger().info("Nav2 Active")

            self.undock_and_start_mission(msg.data)
        else:
            self.navigate_to_point(msg.data)


    def iot_callback_2(self, msg):
        self.get_logger().info(f'IoT Command received: "{msg.data}"')
        if msg.data == 7 :
            self.navigator.cancelTask()
            self.stop_robot()



    
    def undock_and_start_mission(self,command):
        self.navigator.undock()
        self.navigate_to_point(command)


    def navigate_to_point(self, command):
        try:
        # Handle different commands dynamically
            self.get_logger().info(f'Navigating to point based on command: {command}')
            
            if command == 1:
                self.go_to_point_1()  # Define this method to navigate to a specific point
            elif command == 2:
                self.go_to_point_2()
            elif command == 3:
                self.go_to_point_3()
            elif command == 4:
                self.dock_again()
            elif command == 5:
                self.go_to_point_5()            
            else:
                self.get_logger().warn(f'Unknown command: {command}')

        except Exception as e:
            self.get_logger().error(f'Error during navigation: {str(e)}')
        

    def battery_state_callback(self, msg):
            # Update the last known battery level
            self.last_known_battery = msg.percentage * 100  # Convert fraction to percentage
        
    def check_battery_status(self):
        if self.last_known_battery is not None:
            self.get_logger().info(f'Checking battery status: {self.last_known_battery}%')
            if self.last_known_battery <= 12 and not self.navigator.getDockedStatus():
                self.get_logger().info('Battery low: Going to charge.')
                self.dock_again()
        elif self.last_known_battery is None:
            self.get_logger().info('Battery status unknown.')
        else:
            self.get_logger().info(f'Current battery level: {self.last_known_battery}%')

            

def main(args=None):
    rclpy.init(args=args)
    node = CompleteNode()

    # Use a MultiThreadedExecutor to allow concurrent handling of callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
