#!/usr/bin/env python
# for cos, sin, theta
import math
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

"""
#TODO:
follow inner walls, thus follow left if moving counter clockwise

Test in the f1tenth_gym_ros simulator on the 
levine_blocked map by changing the map_path in sim.yaml to 
'maps/levine_blocked'

Halfway down the west hallway a pair of facing door recesses 
opens up; to two beams a doorway looks like a corner, so a 
naive follower steers into it. 
Think about how your node can tell the two apart 
(a corner has a wall ahead, a doorway does not)....

the simulator counts your laps
(/ego_racecar/lap_count, and a "completed lap" line 
in the bridge log).

node must subscribe to /scan and publish 
AckermannDriveStamped on /drive, and it must work when
started with a plain ros2 run wall_follow <executable>:
"""

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')
        """
        Test in the f1tenth_gym_ros simulator on the 
        levine_blocked map by changing the map_path in sim.yaml to 
        'maps/levine_blocked'
        """
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers, as given in lab des
        self.sub_scan=self.create_subscription(LaserScan,lidarscan_topic,self.scan_callback,1)
        self.pub_drive=self.create_publisher(AckermannDriveStamped,drive_topic,1)

        # TODO: set PID gains, from lab des
        self.kp = 1.2
        self.ki = 0.0
        self.kd = 0.2

        # TODO: store history, from lab des
        self.integral = 0.0
        self.prev_error = 0.0 # use prev error for the derivative term
        #deri=(error-self.prev_error)/dt
        self.error = 0.0

        # TODO: store any necessary values you think you'll need
        #velocity, desired distance
        self.velocity=1.5 #highest speed, well adjust the others (1.0,0.5m/s) as a condtibak
        self.desired_dist=1.0 #maintianed from the walls
        #predicted/future distance

        self.lookahead_dist=0.8 # This is the qualirty ahead we predict
        self.theta=math.radians(50.0)# angle between beam a and b

        #limit of steer for car...why? To ensure steering is no irreglaur or extreme
        self.steer_limit=math.radians(30.0)

        #labs edge cases (door and corners)
        self.tresh_dist=0.8 # meauses when a wall/corner is unreable
        self.front_dist=1.0
        #We use the value to tell us from the front side, if a wall/door/corner is near
        
        #track the prev laser beans for the derirvative and state changes edge cases
        self.prev_a=None
        self.prev_b=None
        self.prev_t=None

        #used when we are on the edge case of a doo (it will disregard the 
        #wall state/cycle, and deal with the doorway)
        self.holding=False
        self.hold_start=0.0
        self.hold_time=0.5

        #use these values from the lidar scan, to retrive values for pid
        self.angle_min=0.0
        self.angle_increment=0.0
        self.range_max=0.0
        self.range_min=0.0


    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement 
        at a given angle. 
        Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        #We converts angle values (rads) 
        # into distance meausemnts we may use
        if self.angle_increment==0.0:
            return self.range_max
        #find index
        index=int(round((angle-self.angle_min)/self.angle_increment))

        if index < 0 or index >=len(range_data):
            return self.range_max
            #invalid/inf
        if not np.isfinite(range_data[index]):
            return self.range_max
            #invalid/nan
        
        if range_data[index]<self.range_min or range_data[index]>self.range_max:
            return self.range_max
        
        return range_data[index]

    def get_front(self, range_data): # for edge cases
        #works alot like getrnages
        """
        divide the front of the car into L,C,R values,
        we compare which distance is cloest on each section, and return it
        This allows us to better predict when walls and corners are ahead of us
        """
        #min_dis=float('inf')
        left=float('inf')
        center=float('inf')
        right=float('inf')

        for d in range(-45,46,2): # scan 90 deg in front of the car
            angle=math.radians(d)
            dis=self.get_range(range_data,angle)

            if d<-10:
                right=min(right,dis)
            elif d>10:
                left=min(left,dis)
            else:
                center=min(center,dis)

        return left,center,right
    
    def get_error(self, range_data, dist,curr_time):
        """
        Calculates the error to the wall. 
        Follow the wall to the left (going counter clockwise 
        in the Levine loop).
        You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        """at this point we shoukd have a set of values, all we have to do
        is organize and plug into equations.
        First we meaause a, b, and the centet (L,C,R) values
        """
        theta=self.theta
        right_angle=math.radians(90)
        b=self.get_range(range_data,right_angle)
        a=self.get_range(range_data,(right_angle-theta))
        
        left,center,right=self.get_front(range_data)
        #curr_time=self.get_clock().now().nanoseconds *1e-9
        recent=False
        a_drop=False
        b_drop=False

        """
        this next section relates to detecting doors
        first we verify we are working with recent a,b values

        if we arent in the hold state(stationary) and we didnt detect a door
        we :
        drop a, if it has increased from its last record distance
        drop b "" ^same reson as ablove

        we drop since they are no longer in the lidar FOV
        then we see if a value has droped, and if so we are not
        close to any obseclat, and thus can enter the hold/stationary state where 
        our values (like velcoty) are comstant


        
        """
        if self.prev_t is not None:
            if curr_time-self.prev_t < 0.25:
                recent=True

        if recent and not self.holding:
            if a>self.prev_a+self.tresh_dist:
                a_drop=True

            if b>self.prev_b+self.tresh_dist:
                b_drop=True

            if a_drop or b_drop:
                if center > self.front_dist:
                    self.holding=True
                    self.hold_start=curr_time

        """
        if er are in a hold state, we must check to first make sure we havent 
        "overstayed" our time. So we first check if we've timed out

        if so (time out, or nearing an obsocale), we should break from the hold

        """
        if self.holding:
            timed_out=False

            if curr_time>self.hold_start+self.hold_time:
                timed_out=True

            if center<=self.front_dist or timed_out:
                self.holding=False
            else:
                self.prev_a=a
                self.prev_b=b

                return self.prev_error
        

        # see lab repo for equations
        beta=math.atan2(a*math.cos(theta)-b,
                        a*math.sin(theta))

        curr_dist= b *math.cos(beta)

        lookahead_dist=curr_dist+self.lookahead_dist*math.sin(beta)

        error=lookahead_dist-dist

        self.prev_a=a
        self.prev_b=b
        return error

    def pid_control(self, error, velocity,curr_time):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        #self explatry solve for the p,i,d values
        # TODO: Use kp,get_clock().now().nanoseconds *1e-9

        if self.prev_t is None:
            dt=0.0
        else:
            dt=curr_time-self.prev_t

        # solve for the p term, i term, d terms
        p=self.kp*error

        if dt>0.0:
            self.integral+=error*dt
        self.integral=max(-1.0,min(1.0,self.integral))

        i=self.ki*self.integral

        if dt>0.0:
            deri=(error-self.prev_error)/dt
        else:
            deri=0.0

        d=self.kd*deri
        angle = p + i + d

        angle=max(-self.steer_limit, min(self.steer_limit,angle))

        self.prev_error=error
        self.prev_t=curr_time

        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
        drive_msg.drive.steering_angle=angle
        drive_msg.drive.speed=velocity
        self.pub_drive.publish(drive_msg)


    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
        msg: Incoming LaserScan message
        ki & kd to implement a PID controller
        #curr_time=self.
        Returns:
            None
        """
        self.angle_min=msg.angle_min
        self.angle_increment=msg.angle_increment
        self.range_max=msg.range_max
        self.range_min=msg.range_min

        curr_time=self.get_clock().now().nanoseconds *1e-9

        error = self.get_error(msg.ranges, self.desired_dist,curr_time) 
        # TODO: replace with error calculated by get_error()

        steer_val=abs(self.kp*error)
        if steer_val < math.radians(10):
            velocity=self.velocity
        elif steer_val < math.radians(20):
             velocity=min(self.velocity,1.0)
        else:
            velocity=min(self.velocity,0.5)

        #velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity,curr_time) 
        # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
