import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        # Configuració sèrie (ajusta el port segons la RPi4)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/arm_joints',
            self.listener_callback,
            10)
        self.get_logger().info('Node Serial Bridge de la UB iniciat')

    def listener_callback(self, msg):
        if len(msg.data) == 6:
            # Enviem els valors separats per comes acabats en salt de línia
            data_str = ",".join([str(int(a)) for a in msg.data]) + "\n"
            self.ser.write(data_str.encode('utf-8'))
            self.get_logger().info(f'Angles enviats a Arduino: {data_str.strip()}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
