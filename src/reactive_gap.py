#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

prev_err = 0.0 
integral = 0.0

class reactive_follow_gap:
    def __init__(self):

        #Topics & Subscriptions,Publishers
        drive_topic = '/nav'
        rospy.Subscriber("scan", LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

        # Lidar Parameters
        self.angle_min = 0
        self.angle_max = 0
        self.angle_increment = 0
        self.Ld_angle = []

        # Extracting Parameters form the YAML parameter file
        self.rb = 0.4#rospy.get_param("/reactive/bubble_radius")
        self.view_ang = 60#rospy.get_param("/reactive/view_angle")
        self.thresh = 1.8#rospy.get_param("/reactive/threshold")
        self.cp_blo = 1.5#rospy.get_param("/reactive/cp_threshold")
        self.d_st_ang = 0#rospy.get_param("/reactive/desired_steering_angle")
        self.velocity = [1,1,1]#rospy.get_param("/reactive/velocity")
        self.steer_lim = [15,25]#rospy.get_param("/reactive/steer_angle_limits")

        
        self.kp = 0.7#rospy.get_param("/reactive/kp")
        self.kd = 0#rospy.get_param("/reactive/kd")
        self.ki = 0#rospy.get_param("/reactive/ki")

        rospy.loginfo("Class activated")

    # Call back for Lidar
    def lidar_callback(self, data):

        # Lidar parameters update according to the Lidar
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment

        # Extracting the ranges matrix
        ranges = data.ranges

        # Preprocessing the range matrix
        proc_ranges = self.preprocess_lidar(ranges)
        
        # Getting the index of the closest point in the processed array
        cp_idx = np.where(proc_ranges == np.amin(proc_ranges))[0][0]        

        # Creating the Bubble
        proc_ranges = self.CreateBubble(proc_ranges,cp_idx)

        # Refilter with a certain threshold for a better value
        proc_ranges = self.ThreshFilter(proc_ranges, self.thresh)
        
        # Find max length gap indices
        mg_idx = self.find_max_gap(proc_ranges)

        
        # Find the index of the best point in the gap 
        M_idx = self.find_best_point(mg_idx)

        # Getting the new current steering angle
        M_ang = self.Ld_angle[int(M_idx)]

        # Sending the current angle to the PID controller (Desired is always set to zero)
        st_ang = self.PIDControl(M_ang)

        # Getting the Velocity according to the steering angle
        velocity = self.getVel(st_ang)

        # Publish Drive message
        self.publish_val(st_ang,velocity)
        


    def preprocess_lidar(self, ranges):
        # Getting the index corresponding to the viewing angle parameter
        p_ind = self.getAngleIndex(self.view_ang)
        n_ind = self.getAngleIndex(-self.view_ang)

        # Creating the whole range matrix to keep track of the correct indices
        self.RealAngleMatrix(n_ind,p_ind)

        # Convertinf ranges to array for ease of operations
        ranges = np.array(ranges)

        # Processing range to limit the view of the lidar
        proc_ranges = ranges[n_ind:p_ind]

        # Filtering Nan and Inf
        proc_ranges = self.NanInfFilter(proc_ranges)        

        return proc_ranges


    def getAngleIndex(self,angle):

        # Final angle of the new matrix converted to radians
        new_angle = (angle/360.0)*2*np.pi
        
        # Array Created
        an_array = np.arange(self.angle_min, new_angle, self.angle_increment)
        
        # Index selected by calculating the size of the final angle matrix
        ind = np.size(an_array) 

        return ind


    def RealAngleMatrix(self, n_ind, p_ind):

        # Angle matrix for the whole lidar range
        R_an_array = np.arange(self.angle_min, self.angle_max, self.angle_increment)

        # Updating the general Matrix
        self.Ld_angle = R_an_array[n_ind:p_ind]
    

    
    def NanInfFilter(self, proc_ranges):
        
        # Filtering Nan and Inf for any array
        proc_ranges = proc_ranges[~np.isnan(proc_ranges) & ~np.isinf(proc_ranges)]

        return proc_ranges


    def CreateBubble(self, proc_ranges, cp_idx):

        # Getting the closest point
        cp = proc_ranges[cp_idx]

        # Calculating the angle for bubble
        ang = np.arctan2(self.rb,cp)

        # If the value is Nan, look for more data
        if np.isnan(ang) or np.isinf(ang):
            rospy.Subscriber("scan", LaserScan, self.lidar_callback)
        else:
            pass

        # Calculating the index value from angle
        num_idx = int((ang)/(self.angle_increment))
        
        # Calculating the left and right angle
        l_idx = (cp_idx - num_idx)
        r_idx = (cp_idx + num_idx)

        # Taking care of negative indices
        if l_idx <= 0:
            l_idx = 0
        else:
            pass

        if r_idx <= 0:
            r_idx = 0
        else:
            pass
        
        # Changing the lidar data to represent the bubble
        proc_ranges[l_idx:r_idx] = 0

        # Closest point blowup bubble
        t1 = self.cp_blo*cp
        proc_ranges = self.ThreshFilter(proc_ranges, t1)

        return proc_ranges


    def ThreshFilter(self, proc_ranges, th):

        # Filtering the value with respect to the threshold in the YAML file
        proc_ranges[proc_ranges < th] = 0

        return proc_ranges


    def find_max_gap(self, proc_ranges):

        # Detecting where the zeroes are in the matrix
        zero_mat = np.where(proc_ranges == 0)[0] 

        # Getting an array of arrays showing the zero start and end indices      
        bound_mat = np.split(zero_mat, np.where(np.diff(zero_mat) != 1)[0]+1)

        # Start and End matrix declarations
        strt = []   # start is a collection of all indices where the zeroes start
        end = []    # end is a collection of all indices where the zeroes end

        i = 0
        for i in range(len(bound_mat)):
            strt.append(bound_mat[i][0])
            end.append(bound_mat[i][-1])

        strt.append(len(proc_ranges))   # Putting length of ranges to the end
        end.insert(0, -1)               # Adding -1 to the start for proper gap length calculation

        # Array for all gap lengths
        comp = [0]
        
        # Getting the indices of the max gap
        j = 0
        s_idx = 0
        e_idx = 0
        for j in range(len(strt)):
            sz = strt[j] - end[j] - 1
            comp.append(sz)

            if comp[j+1] > comp[j]:
                s_idx = end[j]
                e_idx = strt[j]

        mg_idx = [s_idx, e_idx]

        return mg_idx

        
    def find_best_point(self, mg_idx):
        
        # Getting the middle index
        start_i = mg_idx[0]        
        end_i = mg_idx[1]
        M_idx = (int(start_i+end_i)/2)

        return M_idx


    def PIDControl(self, M_ang):
        global integral
        global prev_err

        # Error calculation
        err = M_ang - self.d_st_ang

        # Differential term
        diff = err - prev_err

        # Integral term
        integral = integral + err

        # Making the intergral term zero at the crossover
        sign = err*prev_err

        if sign <= 0:
            integral = 0
        else:
            pass

        # Storing error for next loop
        prev_err = err
            
        return self.kp*err + self.kd*diff + self.ki*integral


    def getVel(self,angle):

        # Conversion to dergees for ease of classification
        deg = (180*angle)/(np.pi)
        
        # Choosing velocities
        if abs(deg) <= self.steer_lim[0]:
            vel = self.velocity[2]
        elif abs(deg) > self.steer_lim[0] and abs(deg) <= self.steer_lim[1]:
            vel = self.velocity[1]
        else:
            vel = self.velocity[0]       

        return vel


    def publish_val(self,angle,velocity):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)   

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
