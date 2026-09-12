import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

"""
relay subscribes to the drive topic.
In the subscriber callback, take the speed 
and steering angle from the incoming message, 
multiply both by 3, and publish the new values
via another AckermannDriveStamped message to a 
topic named drive_relay.
"""

class Relay (Node):
    #init
    def __init__(self):
        super().__init__('relay')
        #msg type, topic name, callback, queue size
        self.subscription=self.create_subscription(AckermannDriveStamped,'drive',self.listener_callback,10)

        self.publisher=self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)

    #In the subscriber callback, take the speed  and steering angle from the incoming message
    #So the prev msg must be a parameter for the new msg
    def listener_callback(self,msg):
        new_msg=AckermannDriveStamped()
        # call the talker msg.drive.speed and alter
        new_msg.drive.speed= msg.drive.speed * 3.0
        new_msg.drive.steering_angle= msg.drive.steering_angle * 3.0

        self.publisher.publish(new_msg)

       


def main(args=None):
    rclpy.init(args=args)
    relay=Relay()
    rclpy.spin(relay)
    relay.destroy_node()
    rclpy.shutdown()


if __name__ =="__main__":
    main()

