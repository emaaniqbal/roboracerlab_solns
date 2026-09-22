#!/usr/bin/env python3
"""
lib used: see package.xml
"""
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


"""
Some Req:
- ROS 2 node that subscribes to the LaserScan and Odometry messages (done)
- analyze the LaserScan data and, if necessary,
publish an AckermannDriveStamped with the speed field set to 0.0 m/s to brake


- After you've calculated the array of iTTCs,
you should decide how to proceed with this information.
You'll have to decide how to threshold, and how to best remove false positives
(braking when collision isn't imminent).
Don't forget to deal with infs or nans in your arrays.
"""


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.
        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle
        The subscribers should use the provided odom_callback and scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.0
        self.brake_latch=False
        self.timer=self.create_timer(0.01,self.driver_timer_callback)
        # One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.
        self.publisher_=self.create_publisher (AckermannDriveStamped,
                                               '/drive', 10)
        #You should also subscribe to the /scan topic to get the LaserScan messages and
        #the /ego_racecar/odom topic to get the current speed of the vehicle.
        #The subscribers should use the provided odom_callback and scan_callback as callback methods
        self.scan_subscription=self.create_subscription(LaserScan,
                                                        '/scan',self.scan_callback,10)
        self.odom_subscription=self.create_subscription(Odometry,
                                                        '/ego_racecar/odom',self.odom_callback,10)


    def odom_callback(self, odom_msg):
        #new_msg.xxxx=odom_msg.xxx
        #the longitudinal velocity of the vehicle can be found in twist.twist.linear.x
        self.speed = odom_msg.twist.twist.linear.x


    def scan_callback(self, scan_msg):
        """
        comments:
        #numpy docs: https://numpy.org/doc/stable/reference/routines.math.html
        # ratio of range between wall and obj / time seri of range between car and obj
        # ri=vx cos theta cx= forward moving ref and cos of lidar beam
        # t<=t min as a threshold for when light, mod, or full brake
        # r_arr=[- math.cos(theta) * v_x for theta in theta_arr]
        # #r_arr_negate= [-1 * x for x in r_arr]
        # ref https://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData
        # use np.array
        #first value to calc is the distance, use scan_msg.ranges
        #scan_msg.angle_min and scan_msg.angle_max
        what if r = 0? (infinity), if r is + then ..., if neg..append
        https://numpy.org/doc/2.2/reference/generated/numpy.where.html
        handle false positives?
        itcc_arr= [d/r for d in d_arr for r in r_fin]
        if curr_t  is < threshold ....brake?
         low mid high threshholds
        np,where(condtion, array (this case the div operation), )
        array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
        np.where(a < 5, a, 10*a)
        array([ 0,  1,  2,  3,  4, 50, 60, 70, 80, 90])
        10*a is pushed if cond at start is false
        push infinity...?
        # with np.errstate(divide='ignore', over='ignore', invalid='ignore'):
        # ittc_arr=np.where(r_fin > 0.0, d_arr/r_fin,np.inf)
        # avoid using where be it divides first..lets check r_fin values before div
        # copy array, fill in np.inf as custom value, types are floats
        # https://numpy.org/doc/2.3/reference/generated/numpy.nan_to_num.html
        # send to np.inf
        # finds diff of angles in consecutive beams
        """        
        v_x= self.speed
        if self.brake_latch:
            return
    
        d_arr= np.array(scan_msg.ranges)
        d_arr=np.nan_to_num(d_arr, nan=np.inf, posinf=np.inf, neginf=np.inf,)
        theta_arr=np.linspace(scan_msg.angle_min, scan_msg.angle_max,len(d_arr))

        saftey_thresh= 1.5
        if v_x<0.01:
            return

        r_arr= v_x*np.cos(theta_arr)
        r_fin=np.maximum(r_arr,0.0)
        ittc_arr=np.full_like(d_arr,np.inf,dtype=float)

        valid_range=(d_arr>=scan_msg.range_min) & (d_arr<= scan_msg.range_max)
        valid= (r_fin>0.0) & valid_range
        ittc_arr[valid]= d_arr[valid]/r_fin[valid]

        frnt=np.abs(theta_arr) < np.deg2rad(45.0)
        frnt_ttc=ittc_arr[frnt]
        frnt_dist=d_arr[frnt]

        tcc_brake= frnt_ttc < saftey_thresh
        trigger_brake=tcc_brake & (frnt_dist < 5.0)

        if np.count_nonzero(trigger_brake) >= 3:
            if not self.brake_latch:
                self.get_logger().info("AEB triggered.")
            self.brake_latch=True


    def driver_timer_callback (self):
        # publish brake msg here @ each timer tick
        if self.brake_latch:
            new_msg=AckermannDriveStamped()
            new_msg.header.stamp=self.get_clock().now().to_msg()
            new_msg.header.frame_id='base_link'
            new_msg.drive.speed= 0.0
            new_msg.drive.steering_angle= 0.0
            self.publisher_.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()













