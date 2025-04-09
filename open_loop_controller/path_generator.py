import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import time
from std_msgs.msg import Bool
import sys
import json

class PathGeneratorNode(Node):
    def __init__ (self):
        super().__init__("path_generator")
        self.get_logger().info("Path Generator node has started")

        self.pub = self.create_publisher(Pose, "/next_point", 1)
        self.pub_finished = self.create_publisher(Bool, "/path_finished", 1)
        self.sub_arrived = self.create_subscription(Bool, "/arrived", self.callback_arrived, 10)

        self.create_timer(0.01, self.generate_path)

        self.declare_parameter("point_list", '[ [2.0, 0.0], [2.0, 2.0], [0.0, 2.0], [0.0, 0.0] ]')
        
        self.point_list = json.loads(self.get_parameter("point_list").get_parameter_value().string_value)
        
        self.msg = Pose() 
        self.msg_finished = Bool()
        self.arrived = False
        self.t0 = time.time()
        
    
    def callback_arrived(self, msg):
        if msg.data:
            self.get_logger().info("Arrived at the point")
            self.arrived = True
            self.point_list.pop(0)
        else:
            self.arrived = False

    def generate_path(self):
        if (len(self.point_list) > 0):
            [x, y] = self.point_list[0]
            self.msg.x = x
            self.msg.y = y

            self.pub.publish(self.msg)

        else:
            self.get_logger().info("Path generation finished")
            self.msg_finished.data = True
            self.pub_finished.publish(self.msg_finished)
            self.destroy_node()
            return

        
                    


def main(args=None):
    rclpy.init(args=args)
    node = PathGeneratorNode()
    rclpy.spin(node)
    rclpy.shutdown()
    sys.exit()

if __name__ == "__main__":
    main()