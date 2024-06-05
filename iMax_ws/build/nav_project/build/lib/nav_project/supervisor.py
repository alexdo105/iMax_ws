import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.msg import TransitionEvent
from turtlebot4_msgs.msg import UserLed
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from time import sleep
class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')
        
        # QoS profile for LED messages
        qos_profile_led = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create the LED publisher
        self.led_publisher = self.create_publisher(UserLed, '/hmi/led', qos_profile_led)

        # QoS profile for TransitionEvent messages
        qos_profile_transition_event = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create the subscriber for the transition event
        self.transition_event_subscriber = self.create_subscription(
            TransitionEvent,
            '/velocity_smoother/transition_event',
            self.transition_event_callback,
            qos_profile_transition_event
        )
        self.get_logger().info('SupervisorNode has been initialized and subscribed to /velocity_smoother/transition_event')


    def transition_event_callback(self, msg):
        self.get_logger().info(f'Received TransitionEvent: start_state: {msg.start_state.id} - {msg.start_state.label}, '
                               f'goal_state: {msg.goal_state.id} - {msg.goal_state.label}')
        # Print the values for debugging
        self.get_logger().info(f'goal_state.id: {msg.goal_state.id}, goal_state.label: {msg.goal_state.label}')

        # Check if the goal state is 'active' and the id is 3
        if msg.goal_state.id == 3 and msg.goal_state.label == 'active':
            self.get_logger().info('Velocity smoother is active')
            self.turn_off_leds()
            self.turn_on_noblink_led_left()
            self.turn_on_noblink_led_right()

            
        else:
            self.get_logger().info('Velocity smoother is not active')
            self.handle_node_failure()


    def handle_node_failure(self):
        self.get_logger().info('Handling node failure by turning off the LED...')
        self.turn_off_leds()
        self.red_blink_led_right()

    def turn_on_noblink_led_left(self):
        msg = UserLed()
        msg.led = 0  # LED ID
        msg.color = 1  # Assuming '1' is the color for 'on'
        msg.blink_period = 1000  # No blink, continuous light
        msg.duty_cycle = 1.0  # Fully on
        self.led_publisher.publish(msg)
        self.get_logger().info(f'Published to turn on LED: {msg}')

    def turn_on_noblink_led_right(self):
        msg = UserLed()
        msg.led = 1  # LED ID
        msg.color = 1  # Assuming '1' is the color for 'on'
        msg.blink_period = 1000  # No blink, continuous light
        msg.duty_cycle = 1.0  # Fully on
        self.led_publisher.publish(msg)
        self.get_logger().info(f'Published to turn on LED: {msg}')


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
    
    def red_blink_led_right(self):
        msg = UserLed()
        msg.led = 1  # LED ID for the right LED
        msg.color = 2  # Assuming '2' is the color for 'red'
        msg.blink_period = 1000  # 1 second blink period
        msg.duty_cycle = 0.5  # 50% duty cycle for blinking
        self.led_publisher.publish(msg)
        self.get_logger().info(f'Published to blink right LED in red: {msg}')

def main(args=None):
    rclpy.init(args=args)
    supervisor_node = SupervisorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(supervisor_node)
    try:
        executor.spin()
    finally:
        supervisor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


'''import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from turtlebot4_msgs.msg import UserLed
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')
        self.timer = self.create_timer(5.0, self.check_service_status)
        self.service_name = '/lifecycle_manager_navigation/is_active'
        self.client = self.create_client(Trigger, self.service_name)

        # QoS profile for LED messages
        qos_profile_led = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create the LED publisher
        self.led_publisher = self.create_publisher(UserLed, '/hmi/led', qos_profile_led)

    def check_service_status(self):
        if self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {self.service_name} is available')
            self.turn_off_led_right()
            req = Trigger.Request()
            future = self.client.call_async(req)
            future.add_done_callback(self.service_callback)
        else:
            self.get_logger().error(f'Service {self.service_name} is not available')
            self.handle_node_failure()

    def service_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Service call successful: {response.message}')
                self.turn_on_led()
            else:
                self.get_logger().info(f'Service call failed: {response.message}')
                self.handle_node_failure()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.handle_node_failure()

    def handle_node_failure(self):
        self.get_logger().info('Handling node failure by turning off the LED...')
        self.turn_off_led()
        self.red_blink_led_right()

    def turn_on_led(self):
        msg = UserLed()
        msg.led = 0  # LED ID
        msg.color = 1
        msg.blink_period = 1000
        msg.duty_cycle = 1.0
        self.led_publisher.publish(msg)
        self.get_logger().info(f'Published: {msg}')

    def turn_off_led(self):
        msg = UserLed()
        msg.led = 0  # LED ID
        msg.color = 0 
        msg.blink_period = 0
        msg.duty_cycle = 0.0
        self.led_publisher.publish(msg)
        self.get_logger().info(f'Published: {msg}')

    def turn_off_led_right(self):
        msg = UserLed()
        msg.led = 1  # LED ID
        msg.color = 0 
        msg.blink_period = 0
        msg.duty_cycle = 0.0
        self.led_publisher.publish(msg)
        self.get_logger().info(f'Published: {msg}')
    
    def red_blink_led_right(self):
        msg = UserLed()
        msg.led = 1  # LED ID
        msg.color = 2 
        msg.blink_period = 1000
        msg.duty_cycle = 0.5
        self.led_publisher.publish(msg)
        self.get_logger().info(f'Published: {msg}')

def main(args=None):
    rclpy.init(args=args)
    supervisor_node = SupervisorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(supervisor_node)
    try:
        executor.spin()
    finally:
        supervisor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''