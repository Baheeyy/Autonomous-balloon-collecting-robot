import rclpy
from rclpy.node import Node
from std_msgs.msg import Char, Float32


class MotionTest(Node):
    def __init__(self):
        super().__init__('test_node')
        self.esp_publisher = self.create_publisher(Char, 'esp', 10)
        self.depth_publisher = self.create_subscription(Float32, 'depth', self.check_depth, 10)
        self.get_logger().info('Control Testing Node Has Started!')
        self.have_ball = False
    
    def check_depth(self, msg):
        pass
    #     if msg.data < 5 and self.have_ball is False:
    #         self.closeservo()
    #         self.have_ball = True
    #     elif msg.data < 5 and self.have_ball is True:
    #         self.openservo()
    #         self.have_ball = False


    def move_forward(self):
        msg = Char()
        msg.data = ord('w')
        self.esp_publisher.publish(msg)

    def rotate_right(self):
        msg = Char()
        msg.data = ord('d')
        self.esp_publisher.publish(msg)

    def rotate_left(self):
        msg = Char()
        msg.data = ord('a')
        self.esp_publisher.publish(msg)

    def stop(self):
        msg = Char()
        msg.data = ord('z')
        self.esp_publisher.publish(msg)
    
    def back(self):
        msg = Char()
        msg.data = ord('s')
        self.esp_publisher.publish(msg)

    def stepperup(self):
        msg = Char()
        msg.data = ord('q')
        self.esp_publisher.publish(msg)

    def stepperdown(self):
        msg = Char()
        msg.data = ord('e')
        self.esp_publisher.publish(msg)

    def closeservo(self):
        msg = Char()
        msg.data = ord('y')
        self.esp_publisher.publish(msg)
        # while True:
        #     self.esp_publisher.publish(msg)

    def openservo(self):
        msg = Char()
        msg.data = ord('t')
        self.esp_publisher.publish(msg)
