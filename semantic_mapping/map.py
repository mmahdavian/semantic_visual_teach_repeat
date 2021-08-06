#!/usr/bin/env python
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import PoseStamped,Point32
from sensor_msgs.msg import PointCloud,ChannelFloat32,JointState
import cv2
import numpy as np
import message_filters
from numpy import cos,sin,pi
import csv 
import os

class mapping():
     def __init__(self):
         self.objs = []
         self.obj_map = []
         self.prev_objects = []
         # camera information
         self.im_size_x = 1280 #720 for habitat
         self.im_size_y = 720
         rospy.init_node('listener', anonymous=True)
         self.bb_sub = message_filters.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes)
#         self.cam_sub = message_filters.Subscriber('/keyframe_pose', PoseStamped)
 #        self.cam_sub = message_filters.Subscriber('/orb_pose', PoseStamped)
         self.op_sub = message_filters.Subscriber('/obj_position', PointCloud)
         self.obj_pub = rospy.Publisher('/map_objects', PointCloud , queue_size=1)
         self.ts = message_filters.ApproximateTimeSynchronizer([self.bb_sub,self.op_sub], 10, 0.1)
         self.ts.registerCallback(self.gotdata)
         print("start")

     def gotdata(self,bb,op):

#         big_objs = [59,2,5,8,9,13,20,57,60,61,62,72,80]
#         medium_objs = [0,1,3,9,11,30,31,56,58,63,66,68,69,71,74,77]
#         small_objs = [14,15,16,24,25,26,27,28,35,38,39,40,41,42,43,
#                       44,45,46,47,48,49,54,55,64,65,67,70,73,75,76,79]

         big_objs = [12,13,14,19,20,22]
         medium_objs = [0,1,2,3,4,5,11,15,18,21,23]
         small_objs = [6,7,8,9,10,16,17]

         points = []
         object_position = op.points
         data_op = op.channels[0].values
         for i in range(len(op.points)):
              x_p = object_position[i].x
              y_p = object_position[i].y
              z_p = object_position[i].z
              id_p = data_op[5*i]
              xmin_p = data_op[5*i+1]
              xmax_p = data_op[5*i+2]
              ymin_p = data_op[5*i+3]
              ymax_p = data_op[5*i+4]
              if id_p == 23:
                 print("clock at: "+ str([x_p,y_p,z_p]))      

              details_obj = [x_p,y_p,z_p,id_p,0,xmin_p,xmax_p,ymin_p,ymax_p]
              points.append(details_obj)
         self.prev_objects = points
 
         if(len(self.objs)==0):
              self.objs = np.array(points)
              return 0

         else:
              for i in range(len(points)):
                   matched = 0
                   new_obj = np.array(points[i])
                   cur_x_mid = (new_obj[5]+new_obj[6])/2
                   cur_y_mid = (new_obj[7]+new_obj[8])/2
                   for j in range(len(self.objs)):
                       prev_obj = self.objs[j,:]
                       prev_x_mid = (prev_obj[5]+prev_obj[6])/2
                       prev_y_mid = (prev_obj[7]+prev_obj[8])/2
                       if prev_obj[3] == new_obj[3]:
                            dist = np.sqrt((prev_obj[0]-new_obj[0])**2 + (prev_obj[1]-new_obj[1])**2 + (prev_obj[2]-new_obj[2])**2)
                            if dist<0.5:
                                 self.objs[j,0] = new_obj[0]
                                 self.objs[j,1] = new_obj[1]
                                 self.objs[j,2] = new_obj[2]
                                 self.objs[j,4] = self.objs[j,4] + 1
                                 matched = 1

                            if matched == 0 and abs(cur_x_mid-prev_x_mid) < self.im_size_x/15 and abs(cur_y_mid-prev_y_mid) < self.im_size_y/15:
                                 print("found similar")
                                 self.objs[j,0] = new_obj[0]
                                 self.objs[j,1] = new_obj[1]
                                 self.objs[j,2] = new_obj[2]
                                 self.objs[j,4] = self.objs[j,4] + 1
                                 matched = 1

                   if matched == 0:
                        self.objs = np.vstack((self.objs,np.array(new_obj)))
              if(len(self.obj_map) == 0):
                   my_obj = self.objs
                   for i in range(len(self.objs)-1,-1,-1):
                        if(self.objs[i,4]>2):
                             self.obj_map.append(self.objs[i,:])
                             self.objs = np.delete(self.objs,i,0)

              else:
                   self.objs = np.array(self.objs)
                   self.obj_map = np.array(self.obj_map)
                   for i in range(len(self.objs)-1,-1,-1):
                        if(self.objs[i,4]>2):
                             all_matched = 0
                             for j in range(len(self.obj_map)-1,-1,-1):

                                  if self.obj_map[j,3] == self.objs[i,3]:
                                       dist2 = np.sqrt((self.obj_map[j,0]-self.objs[i,0])**2 + (self.obj_map[j,1]-self.objs[i,1])**2 + (self.obj_map[j,2]-self.objs[i,2])**2)
                                       if self.obj_map[j,3] in big_objs:
                                           thresh = 1.5
                                       elif self.obj_map[j,3] in medium_objs: 
                                           thresh = 0.75
                                       else:
                                           thresh = 0.5

                                       if dist2<thresh:
                                            all_matched = 1
                                            self.objs = np.delete(self.objs,i,0)
                                            break

                             if all_matched == 0:                                
                                  self.obj_map = np.vstack((self.obj_map,self.objs[i,:]))
                                  self.objs = np.delete(self.objs,i,0)
     
         print("all objects are: "+str(self.objs))
         print("all obj_maps are: "+str(self.obj_map))
         print()
         cloud = PointCloud();
         classes = ChannelFloat32()
         geo_points = []
         obj_classes = []
         for i in range(len(self.obj_map)):
              cur_object = self.obj_map[i]
              if (len(self.obj_map) != 0):
                   pt = Point32()
                   pt.x = cur_object[0]
                   pt.y = cur_object[1]
                   pt.z = cur_object[2]
                   geo_points.append(pt)
                   obj_classes.append(cur_object[3])
         classes.values = np.array(obj_classes)
         classes.name = "objects in map"
         cloud.header.frame_id = "map"
         cloud.header.stamp = rospy.Time.now()
         cloud.points = geo_points
         cloud.channels = [classes]
         self.obj_pub.publish(cloud)
         with open('/home/mohammad/Desktop/obj_map.csv', mode='w') as map1:
             map_writer = csv.writer(map1, delimiter=',')
             for obj in self.obj_map:
                 map_writer.writerow([obj[0],obj[1],obj[2],obj[3]])


if __name__ == '__main__':
    path_file = '/home/mohammad/Desktop/obj_map.csv'
    if os.path.exists(path_file):
        os.remove(path_file)
    mapping()
    rospy.spin()
