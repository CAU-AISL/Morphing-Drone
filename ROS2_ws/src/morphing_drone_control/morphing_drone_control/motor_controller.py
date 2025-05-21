from std_msgs.msg import Float32MultiArray

class MotorController:
    def __init__(self, node, state):
        self._state = state  
        self._node = node
        
        self._motor_pub = node.create_publisher(Float32MultiArray, '/motor_wab', 10)

    def send_commands(self):
        data = []
        data += [float(x) for x in self._state.w_d.flatten()]
        data += [float(a) for a in self._state.alpha.flatten()]
        data += [float(b) for b in self._state.beta_dot.flatten()]
        
        msg = Float32MultiArray()
        msg.data = data
        self._motor_pub.publish(msg)
        self._node.get_logger().info(f"[motor_controller] publish wab: {msg.data}")