
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from .tk_litter_world_thread import TkLitterWorldThread

from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool
     

class LitterWorldController(Node):

    def __init__(self, tk_litter_world_thread):
        super().__init__("litter_world") 

        self.counter_ = 0

        self.tk_litter_world_thread = tk_litter_world_thread

        self.subscriber_ = self.create_subscription(Int64, "number", self.callback_number_publisher, 10)

        self.publisher_ = self.create_publisher(Int64, "number_count", 10) # create topic number_count in which number_counter will publish the count
        
        # self.server_ = self.create_service(SetBool, "reset_counter", self.callback_reset_counter)
        
        self.get_logger().info("Litter world has been started")

    def callback_number_publisher(self, msg):
        self.counter_ += msg.data
        self.tk_litter_world_thread.update_counter(self.counter_)
        self.get_logger().info("Received from /number: {}".format(msg.data))
        msg = Int64()
        msg.data = self.counter_
        self.publisher_.publish(msg)

    # def callback_reset_counter(self, request, response):
    #     if(request.data):
    #         self.counter_ = 0
    #         response.success = True
    #         response.message = "Counter reset to 0"
    #     return response

def main(args=None):
    rclpy.init(args=args) # first line to be written in any ROS2 .py node
    # Create new threads
    tk_litter_world_thread = TkLitterWorldThread()

    # Start new Threads
    tk_litter_world_thread.start()
    node = LitterWorldController(tk_litter_world_thread)
    rclpy.spin(node) # allow node to continue to be alive
    rclpy.shutdown() # last line of any ROS2 .py node
    tk_litter_world_thread.join()

if __name__ == "__main__":
    main()