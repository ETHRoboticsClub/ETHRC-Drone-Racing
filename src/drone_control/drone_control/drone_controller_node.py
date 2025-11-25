import rclpy
from rclpy.node import Node, Timer, Publisher
from std_msgs.msg import Float64MultiArray

from drone_control.mavsdk_client import MavSDKClient
import numpy as np

class DroneController(Node):

    mav_sdk_client : MavSDKClient

    control_rate: float
    state_update_rate: float
    control_timer: Timer
    state_update_timer: Timer
    state_publisher: Publisher

    state: np.ndarray  = np.zeros(13) #TODO: maybe change to state class

    #TODO: add control


    def __init__(self):
        super().__init__('drone_controller_node')

        self.mav_sdk_client = MavSDKClient()
        self.declare_parameter('controller_rate',100.0)
        self.declare_parameter('state_update_rate',200.0)

        self.control_rate = self.get_parameter('controller_rate').get_parameter_value().double_value
        self.state_update_rate = self.get_parameter('state_update_rate').get_parameter_value().double_value

        #TODO: maybe change to joint_states
        self.state_publisher = self.create_publisher(Float64MultiArray, '/drone_state', 10)

    def initialize(self):
        """ Constructor should return fast, so async operations are moved here. """
        init_future = self.mav_sdk_client.run_async(self.mav_sdk_client.initialize())
        # Do any other time consuming job

        init_future.result() #TODO: handle failure

        # start timers after connecting to mavsdk
        self.control_timer = self.create_timer(1.0/self.control_rate, self._control_timer_callback)
        self.state_update_timer = self.create_timer(1.0/self.state_update_rate, self._state_timer_callback)
        
    def _state_timer_callback(self):
        """This function publishes the drone system to be available in the ros ecosystem"""
        self.state =  self.mav_sdk_client.get_latest_state()
        msg = Float64MultiArray()
        msg.data = self.state.tolist()
        self.state_publisher.publish(msg)

    def _control_timer_callback(self):
        """This function publishes the drone commands immediately through the mav sdk"""
        # TODO: do control
        print(f"State is {self.state}")        


def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    node.initialize()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()