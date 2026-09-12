import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


"""
Talker.py
Deliverable 2: create two nodes in
the package we just created.
You can use either Python or C++ for these nodes.


The first node will be named
talker.cpp or talker.py and needs to meet
these criteria:


talker listens to two ROS parameters v and d.
talker publishes an AckermannDriveStamped
message with the speed field equal to the v
parameter and steering_angle field equal to
the d parameter, and to a topic named drive.


talker publishes as fast as possible.


To test node, set the two ROS parameters
through command line, a launch file, or a
yaml file.
"""
# rclpy.spin(node) keeps script running
# rclpy.shutdown() turns off node?
# sym links to stop building again and again..?
# set para: self.declare_prataemer (name, defalut value)
# get para: my_para= self.get_parameter('my_para'). value


# ref:
# https://docs.ros.org/en/galactic/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html


class Talker (Node):
    #init
    def __init__(self):
        super().__init__('talker')
        #default d, v values
        self.declare_parameter('d',0.0)
        self.declare_parameter('v',0.0)
        #publisher
        #msg=ackerman, topic=drive, queue size
        self.publisher=self.create_publisher(AckermannDriveStamped, 'drive', 10)


        #timer
        quick_time=0.01
        self.timer= self.create_timer(quick_time,self.timer_callback)


    def timer_callback(self):
        """timer callback"""
        msg=AckermannDriveStamped()
        """
        speed field equal to the v
        parameter and
        steering_angle field equal to
        the d parameter
        """
        msg.drive.speed= self.get_parameter('v').value
        msg.drive.steering_angle=self.get_parameter('d').value


        #publish msg
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    talker=Talker()
    rclpy.spin(talker)


    #destroy the node
    talker.destroy_node()
    rclpy.shutdown()


if __name__ =="__main__":
    main()

