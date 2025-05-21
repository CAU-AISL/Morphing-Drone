from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


class MotorController:
    def __init__(self, node, state):
        self._state = state  
        self._node = node
        qos_profile = QoSProfile(depth=10)
        
        self.wab_pub = self._node.create_publisher(Float32MultiArray, '/motor_wab', qos_profile)
        self.timer = self._node.create_timer(0.1, self.send_commands)
        
        

    def send_commands(self):
        data = []
        data += [float(x) for x in self._state.w_d.flatten()]
        data += [float(a) for a in self._state.alpha.flatten()]
        data += [float(b) for b in self._state.beta_dot.flatten()]
        
        msg = Float32MultiArray()
        msg.data = data
        self.wab_pub.publish(msg)
        self._node.get_logger().info(f"[motor_controller] publish wab: {msg.data}")