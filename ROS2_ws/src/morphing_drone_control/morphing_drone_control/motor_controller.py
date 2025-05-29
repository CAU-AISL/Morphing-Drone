from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


class MotorController:
    def __init__(self, node, state, drone_model):
        self._state = state  
        self._node = node
        self.drone_model = drone_model
        qos_profile = QoSProfile(depth=10)
        
        self.wab_pub = self._node.create_publisher(Float32MultiArray, '/motor_wab', qos_profile)
        dt = 0.01 # self.drone_model.current_time-self.drone_model.prev_time
        self.timer = self._node.create_timer(dt, self.send_commands)
        
        

    def send_commands(self):
        data = []
        data += [float(x) for x in self._state.w_d.flatten()]
        data += [float(a) for a in self._state.alpha.flatten()]
        data += [float(b) for b in self._state.beta_dot.flatten()]
        
        msg = Float32MultiArray()
        msg.data = data
        self.wab_pub.publish(msg)
        self._node.get_logger().info(f"[motor_controller] publish wab: {msg.data}")