#!/usr/bin/env python
import numpy as np
import csv 
import rospy
from sensor_msgs.msg import PointCloud,ChannelFloat32
from geometry_msgs.msg import PoseStamped,Point32,PoseArray,Pose
from scipy.spatial.transform import Rotation as R
import time
import os
import math
import matplotlib.pyplot as plt
from numpy import pi,cos,sin
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger,TriggerResponse
import copy

class controller():
    def __init__(self,teach_map,camera1):

         self.init_mode = True
         self.path1 = camera1
         self.cp_idx = 0
         self.path_angles = []
         self.current_euler = 0
         self.tot_path = None
         self.repeat_done = False
         self.start_position = None 
         self.T_changed = False

         rospy.init_node('listener', anonymous=True)
         self.pose_sub = rospy.Subscriber('/new_orb_pose', PoseStamped, self.get_pose)
         self.velocity_publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
         self.trig = rospy.Service('/new_loc',Trigger,self.trigger)
         self.goal_points_publisher = rospy.Publisher('/goal_points_pose', PointCloud , queue_size=1)
         self.nodrift_cam_pub = rospy.Publisher('/new_nodrift_cam_pub', PoseStamped , queue_size=1)

         while not rospy.is_shutdown():
            if self.init_mode == True:
                while self.start_position == None:
                    time.sleep(1)
                    self.move_init(self.start_position)
            tot_path,tot_angles = self.follow_teach(camera1)
            if self.repeat_done == True:
                return

    def move_init(self,start_position): 
        #### go to closest point on teach path
        new_position = [start_position[0],start_position[1]]
        closest_point,idx = self.closest_to_teach(new_position)
        self.closest_point = closest_point
        self.cp_idx = idx
        start = [new_position[0], new_position[1]]
        goal = [closest_point[0], closest_point[2]]
            
        init_path = []
        init_path.append(start)
        init_path.append(goal)

        self.init_mode = False
        self.tot_path = init_path
        tot_path,tot_angles = self.follow_teach(camera1)


    def closest_to_teach(self,cur_pose):
        min_error = 100
        idx = 0
        for i,point in enumerate(self.path1):
            error = np.linalg.norm([cur_pose[0]-point[0],cur_pose[1]-point[2]])
            if error<min_error:
                closest_point = point
                min_error = error
                idx = i
        closest_point=self.path1[idx+50]
        idx = idx+50
        return closest_point,idx

    def motion_planning(self,x1,y1,yaw1,v1,x2,y2,yaw2,v2):
        x1 = 0
        y1 = 0
        yaw1 = 0.
        v1 = 0.
        vx1 = v1 * cos(yaw1)
        vy1 = v1 * sin(yaw1) 
        t1 = 0.
        #
        x2 = 0
        y2 = 0
        yaw2 = pi/2 
        v2 = 1
        vx2 = v2 * cos(yaw2)
        vy2 = v2 * sin(yaw2) 
        t2 = 10.
        #
        
        A1 = np.array([[1,t1,t1**2,t1**3],
                       [0,1,2*t1,3*t1**2],
                       [1,t2,t2**2,t2**3],
                       [0,1,2*t2,3*t2**2]])
        
        A2 = np.array([[1,t1,t1**2,t1**3],
                       [0,1,2*t1,3*t1**2],
                       [1,t2,t2**2,t2**3],
                       [0,1,2*t2,3*t2**2]])
        
        B1 = np.array([[x1],
                       [vx1],
                       [x2],
                       [vx2]])
        
        B2 = np.array([[y1],
                       [vy1],
                       [y2],
                       [vy2]])
        
        X = np.dot(np.linalg.inv(A1),B1).T
        Y = np.dot(np.linalg.inv(A2),B2).T
        
        dt = 0.1
        
        X_pos = [x1]
        Y_pos = [y1]
        V_X = [vx1]
        V_Y = [vy1]
        Teta = [yaw1*180/pi]
        tot_v = [v1]
        W = [0]
        
        for T in np.arange(t1+dt,t2+dt,dt): 
            S = np.array([[1],
                          [T],
                          [T**2],
                          [T**3]])            
            x = np.dot(X[0],S)
            X_pos.append(x[0])
            y = np.dot(Y[0],S)
            Y_pos.append(y[0])
            x_dot = (X_pos[-1] - X_pos[-2])/dt
            y_dot = (Y_pos[-1] - Y_pos[-2])/dt
            V_X.append(x_dot)
            V_Y.append(y_dot)
            theta = np.arctan2(y_dot,x_dot)
            if theta<0:
                theta += 2*pi
            Teta.append(theta*180/pi)
            if abs(Teta[-1] - Teta[-2]) > 10:
                omega = W[-1]
            else:
                omega = (Teta[-1] - Teta[-2])/dt
            W.append(omega)
            v = np.sqrt(x_dot**2 + y_dot**2)
            tot_v.append(v)
        
        V_X[-1] = vx2
        V_Y[-1] = vy2
        tot_v[-1] = v2
        Teta[-1] = yaw2*180/pi

        return tot_v,W,Teta,X_pos,Y_pos

    def follow_teach(self,camera1):
        vel_msg = Twist()
        X = []
        Y = []
        while(self.tot_path == None):
            time.sleep(1)
 
        for i in range(self.cp_idx,len(self.path1)):
            point = self.path1[i] 
            if (self.cp_idx - i)%100 == 0:
                path_point = [point[0],point[2]]
                self.tot_path.append(path_point)
        self.tot_path.append([self.path1[-1][0],self.path1[-1][2]]) 
        for i in range(len(self.tot_path)-1):
            dx = self.tot_path[i+1][0] - self.tot_path[i][0]
            dy = self.tot_path[i+1][1] - self.tot_path[i][1]
            X.append(self.tot_path[i][0])
            Y.append(self.tot_path[i][1]) 
            print(X)
            print(Y)           

            angle = np.arctan2(dy,dx)
            if angle<0:
                angle += 2*np.pi
            self.path_angles.append(angle*180/np.pi)

        #### publish goal points
        goal_points_cloud = PointCloud();
        goal_points = []
        for j in range(len(self.tot_path)):
            gp = self.tot_path[j]
            pt = Point32()
            pt.x = gp[0]
            pt.y = 0
            pt.z = gp[1]
            goal_points.append(pt)

        goal_points_cloud.header.frame_id = "map"
        goal_points_cloud.header.stamp = rospy.Time.now()
        goal_points_cloud.points = goal_points
        goal_points_cloud.channels = []
        self.goal_points_publisher.publish(goal_points_cloud)

        drift_x = 0
        drift_z = 0

        print("planning") 
        for i in range(len(self.tot_path)):
            if self.T_changed == True:
                self.init_mode == True
                self.T_changed == False
                return

            dx = self.tot_path[i+1][0] - (self.xp) 
            dz = self.tot_path[i+1][1] - (self.zp)
            goal_yaw = np.arctan2(dx,dz)

            if goal_yaw<0:
                goal_yaw += 2*np.pi
            goal_yaw = goal_yaw *180/np.pi
            dis_yaw = goal_yaw - self.current_euler
            if dis_yaw > 0:
                sign = -1
            else:
                sign = 1
            if dis_yaw > 180:
                sign = 1
            if dis_yaw < -180:
                sign = -1
                
            dis_yaw_thresh = 3
            
            self.xp_cons = copy.deepcopy(self.xp)
            self.zp_cons = copy.deepcopy(self.zp)

            init_angle = self.current_euler
            while(abs(dis_yaw) > dis_yaw_thresh):
                if self.T_changed == True:
                    self.init_mode == True
                    self.T_changed == False
                    return

                print("prev_p: "+str(self.tot_path[i])+"\n"+"next:"+str(self.tot_path[i+1])+"\n"+"cur:"+str([self.xp,self.zp]))
                dx = self.tot_path[i+1][0] - self.xp
                dz = self.tot_path[i+1][1] - self.zp

                goal_yaw = np.arctan2(dx,dz)
                if goal_yaw<0:
                    goal_yaw += 2*np.pi
                goal_yaw = goal_yaw *180/np.pi
                dis_yaw = goal_yaw - self.current_euler
                print("yaw goal is: " +str(goal_yaw)+" current yaw is: "+str(self.current_euler)+" dis_yaw is: "+str(dis_yaw))
                if dis_yaw > 0:
                    sign = -1
                else:
                    sign = 1
                if dis_yaw > 180:
                    sign = 1
                if dis_yaw < -180:
                    sign = -1
  
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0

                vel_msg.angular.z = sign * 0.45 # check the sign later
                self.velocity_publisher.publish(vel_msg)

                if abs(init_angle-self.current_euler)>25:       
                     print("waiting for ORB-SLAM")
                     time.sleep(4)
                     init_angle = self.current_euler


                nodrift_cam = PoseStamped()
                nodrift_cam.header.stamp = rospy.Time.now()
                nodrift_cam.header.frame_id = "map"
                nodrift_cam.pose.position.x = self.xp
                nodrift_cam.pose.position.y = self.yp
                nodrift_cam.pose.position.z = self.zp

                nodrift_cam.pose.orientation.x = self.xa
                nodrift_cam.pose.orientation.y = self.ya
                nodrift_cam.pose.orientation.z = self.za
                nodrift_cam.pose.orientation.w = self.wa 

                self.nodrift_cam_pub.publish(nodrift_cam)


            drift_x = copy.deepcopy(self.xp) - self.xp_cons
            drift_z = copy.deepcopy(self.zp) - self.zp_cons

            print("reached the angle")
            time.sleep(2)
                
            dis_p = np.linalg.norm([self.tot_path[i+1][0] - self.xp,self.tot_path[i+1][1] - self.zp]) 

            dis_p_thresh = 0.05
            while(dis_p > dis_p_thresh):
                if self.T_changed == True:
                    self.init_mode == True
                    self.T_changed == False
                    return

                dx = self.tot_path[i+1][0] - (self.xp )
                dz = self.tot_path[i+1][1] - (self.zp )
                goal_yaw = np.arctan2(dx,dz)
                if goal_yaw<0:
                    goal_yaw += 2*np.pi

                goal_yaw = goal_yaw * 180/np.pi
                dis_yaw = goal_yaw - self.current_euler
                print("goal: " +str(self.tot_path[i+1])+" current: "+str([self.xp,self.zp])+" dis: "+str(dis_p)+" dis_yaw:"+str(dis_yaw))

                dis_p = np.linalg.norm([self.tot_path[i+1][0] - (self.xp),self.tot_path[i+1][1] - (self.zp)])

                vel_msg.linear.x = 0.1
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                if abs(dis_yaw)>dis_yaw_thresh:

                    if dis_yaw > 0:
                       sign = -1
                    else:
                       sign = 1
                    if dis_yaw > 180:
                       sign = 1
                    if dis_yaw < -180:
                       sign = -1

                    print("dis yaw is: "+str(dis_yaw))
                    vel_msg.angular.z = sign*0.2
                    print("angle cmd: " +str(sign*0.2))

                else:
                    vel_msg.angular.z = 0
                              
                self.velocity_publisher.publish(vel_msg)

                nodrift_cam = PoseStamped()
                nodrift_cam.header.stamp = rospy.Time.now()
                nodrift_cam.header.frame_id = "map"
                nodrift_cam.pose.position.x = self.xp 
                nodrift_cam.pose.position.y = self.yp
                nodrift_cam.pose.position.z = self.zp 



                nodrift_cam.pose.orientation.x = self.xa
                nodrift_cam.pose.orientation.y = self.ya
                nodrift_cam.pose.orientation.z = self.za
                nodrift_cam.pose.orientation.w = self.wa 

                self.nodrift_cam_pub.publish(nodrift_cam)

            print("reached a point")
            time.sleep(1)
            #After the loop, stops the robot
            vel_msg.linear.x = 0
            #Force the robot to stop
            self.velocity_publisher.publish(vel_msg)
            self.repeat_done = True
        
        return self.tot_path,self.path_angles
        
    def get_pose(self,pose):
        self.xp = pose.pose.position.x
        self.yp = pose.pose.position.y
        self.zp = pose.pose.position.z
        self.xa = pose.pose.orientation.x
        self.ya = pose.pose.orientation.y
        self.za = pose.pose.orientation.z
        self.wa = pose.pose.orientation.w
        r2 = R.from_quat([self.xa,self.ya,self.za,self.wa])
        yaw_angle = r2.as_euler('XZY')
        self.current_euler = yaw_angle[2] *180/np.pi
        if self.current_euler < 0:
            self.current_euler += 360
        self.start_position = [self.xp,self.zp]
        start_euler = [self.current_euler]

    def trigger(self,req):
        self.T_changed = True
        print("trigger called")
        return TriggerResponse(True,"hi")

if __name__ == '__main__':

#    plt.figure()    

    camera1 = []
    teach_map = []

    with open('/home/mohammad/Desktop/C1.csv', mode = 'r') as cam1:
        teach_cam = csv.reader(cam1, delimiter=',',quoting=csv.QUOTE_NONNUMERIC)
        for row in teach_cam:
            camera1.append(row)


    with open('/home/mohammad/Desktop/M1.csv', mode = 'r') as map1:
        teach_csv = csv.reader(map1, delimiter=',',quoting=csv.QUOTE_NONNUMERIC)
        for row in teach_csv:
            teach_map.append(row)

    print(teach_map)
    controller(teach_map, camera1)
    rospy.spin()

