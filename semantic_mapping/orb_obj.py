#!/usr/bin/env python
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import PoseStamped,Point32
from sensor_msgs.msg import PointCloud,ChannelFloat32
import cv2
import numpy as np
import message_filters
from scipy import stats
import csv
import os

class obj_position():
     def __init__(self):
         self.prev_cam=np.array([0,0,0])    

         # camera information
         self.im_size_x = 1280  #720 for habitat
         self.im_size_y = 720
         self.K = np.array([[528.96002, 0, 620.22998],
                            [0, 528.66498, 368.64499],
                            [0,      0   ,     1    ]])  # camera K matrix

         self.founded_objects = PointCloud()
         self.object_class = ChannelFloat32()

         rospy.init_node('listener', anonymous=True)
         self.bb_sub = message_filters.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes)
         self.cam_sub = message_filters.Subscriber('/orb_pose', PoseStamped)
         self.pc_sub = message_filters.Subscriber('/point_cloud', PointCloud )
         self.obj_pub = rospy.Publisher('/obj_position', PointCloud , queue_size=1)         
         self.ts = message_filters.ApproximateTimeSynchronizer([self.bb_sub,self.cam_sub,self.pc_sub], 1, 0.02)
         self.ts.registerCallback(self.gotdata)         

     def clean_overlap(self,points_cl_mid):
         
         if len(points_cl_mid) != 0:
              check_mid = points_cl_mid[:,0:3]
              res_mid =np.unique(check_mid,return_counts = True,axis = 0)
              counts_mid = res_mid[1]
              bigger_than_one_mid = np.where(counts_mid>1)
              bigger_than_one_mid = bigger_than_one_mid[0]
              if len(bigger_than_one_mid)>0:
                   for i in bigger_than_one_mid:
                        delete_list_mid = np.where((points_cl_mid[:,0:3] == res_mid[0][i]).all(axis=1))
                        delete_list_mid = delete_list_mid[0]
                   delete_list_mid = delete_list_mid.tolist()
                   delete_list_mid.reverse()
                   for k in delete_list_mid:
                        points_cl_mid = np.delete(points_cl_mid,k,0)
         return points_cl_mid 

     def gotdata(self,bound,cam_pose,pc):
      #   print("worked")
         points_cat = []
         points_cat_mid = []
         objects = []
         objects_info = []

         c=np.zeros(3)
         R=np.zeros(4)
         
         c[0] = cam_pose.pose.position.x
         c[1] = cam_pose.pose.position.y
         c[2] = cam_pose.pose.position.z

         R[0] = cam_pose.pose.orientation.x
         R[1] = cam_pose.pose.orientation.y
         R[2] = cam_pose.pose.orientation.z
         R[3] = cam_pose.pose.orientation.w
         
         with open('/home/mohammad/Desktop/projection.csv', mode='a') as proj_file:
             proj_writer = csv.writer(proj_file, delimiter=',')
             proj_writer.writerow([c[0],c[1],c[2],R[0],R[1],R[2],R[3]])

         K = self.K 
         Rot = np.array([[1-2*(R[1]**2)-2*(R[2]**2),2*R[0]*R[1]+2*R[2]*R[3],2*R[0]*R[2]-2*R[1]*R[3],0],
                         [2*R[0]*R[1]-2*R[2]*R[3],1-2*(R[0]**2)-2*(R[2]**2),2*R[1]*R[2]+2*R[0]*R[3],0],
                         [2*R[0]*R[2]+2*R[1]*R[3],2*R[1]*R[2]-2*R[0]*R[3],1-2*(R[0]**2)-2*(R[1]**2),0],
                         [0,0,0,1]])

         Proj = np.zeros((3,4))
         Proj[0:3,0:3] = Rot[0:3,0:3]
         tcw_ros = np.array([[-c[0]],[-c[1]],[-c[2]]])
         tcw_orb = np.dot(Rot[0:3,0:3],tcw_ros)

         Proj[0,3]=tcw_orb[0]
         Proj[1,3]=tcw_orb[1]
         Proj[2,3]=tcw_orb[2]          
         cur_cam = cam_pose.pose.position
         
         self.Projection = np.dot(K,Proj)
         d=np.array([self.prev_cam[0]-cur_cam.x,self.prev_cam[1]-cur_cam.y,self.prev_cam[2]-cur_cam.z])
         self.prev_cam = np.array([cur_cam.x,cur_cam.y,cur_cam.z])
         geo_points = pc.points
         channels = pc.channels
         obj = 0

         for bb in bound.bounding_boxes:
              current_time = rospy.Time.now()
              ins = []
              xmin = bb.xmin
              xmax = bb.xmax
              ymin = bb.ymin
              ymax = bb.ymax
              Class = bb.Class
              id_obj = bb.id
              prob = bb.probability
              if Class == "None":
                   obj = obj - 1
                   continue
              obj = obj + 1
              xmid = (xmin+xmax)/2
              ymid = (ymin+ymax)/2
              print("I saw a "+Class)
              if xmin < self.im_size_x/25 or xmax > 24*self.im_size_x/25   or ymin < self.im_size_y/25:
                   continue          
              print("considered "+Class)

              for i in range(len(geo_points)):
                   fx = geo_points[i].x
                   fy = geo_points[i].y
                   fz = geo_points[i].z
                   pt3d = np.array([[fx],[fy],[fz],[1]])
                   pt2d = np.dot(self.Projection,pt3d)
                   pt2d[0] = pt2d[0] / pt2d[2]
                   pt2d[1] = pt2d[1] / pt2d[2]
                   pt2d[2] = pt2d[2] / pt2d[2]
             
                   if pt2d[0]<xmax and xmin<pt2d[0] and pt2d[1]<ymax and ymin<pt2d[1]:
                        dis_p_v = np.array([xmid - pt2d[0],ymin - pt2d[1]])
                        dis_p = np.linalg.norm(dis_p_v)
                        in_bb = [dis_p,fx,fy,fz]
                        ins.append(in_bb)

              ins = np.array(ins)
              if len(ins)>0:
                  median_z = np.median(ins[:,3])  # z is forward
                  for i in range(len(ins)-1,-1,-1):
                      if (ins[i,3]-median_z) > 0.5:
                          ins = np.delete(ins,i,axis=0)
                                  
              if len(ins) == 0:
                   continue
              ins = ins[ins[:,0].argsort()]
              if len(ins)>3:
                   closests = ins[0:3]
              else:
                   closests = ins
              closests = np.array(closests)

              print("closests are\n"+str(closests))

              for j in range(len(closests)-1,0,-1):
                  dc_v = [closests[j,1]-closests[0,1] ,closests[j,2]-closests[0,2], closests[j,3]-closests[0,3]]
                  dc = np.linalg.norm(dc_v)
                  if dc>1:
                      closests = np.delete(closests,j,axis=0)

              for j in range(len(closests)-1,-1,-1):
                  if closests[j,0] > 30: 
                      closests = np.delete(closests,j,axis=0) 
              if len(closests) == 0:
                  continue

              Xs = closests[:,1]
              Ys = closests[:,2]
              Zs = closests[:,3]
              X = np.mean(Xs)
              Y = np.mean(Ys)
              Z = np.mean(Zs)
              points = [X,Y,Z,Class,obj,id_obj,xmid,ymid,xmin,xmax,ymin,ymax]
              points_cat.append(points)

         points_cl = np.array(points_cat)
         for i in range(len(points_cl)):
              if(len(points_cl)==0):
                   break

              print("saw a "+points_cl[i,3]+ " at: "+str([float(points_cl[i,0]),float(points_cl[i,1]),float(points_cl[i,2])]))
              object_position = Point32()
              object_position.x = float(points_cl[i,0])
              object_position.y = float(points_cl[i,1])
              object_position.z = float(points_cl[i,2])
              objects.append(object_position)
              objects_info.append(float(points_cl[i,5]))
              objects_info.append(float(points_cl[i,8]))
              objects_info.append(float(points_cl[i,9]))
              objects_info.append(float(points_cl[i,10]))
              objects_info.append(float(points_cl[i,11]))
              self.object_class.values = np.array(objects_info)
              self.object_class.name = "objects"

         self.founded_objects.points = objects
         self.founded_objects.channels = [self.object_class]
         self.founded_objects.header.stamp = rospy.Time.now()
         self.founded_objects.header.frame_id = "map"
         self.obj_pub.publish(self.founded_objects)
         
if __name__ == '__main__':
    path_file = '/home/mohammad/Desktop/projection.csv'
    if os.path.exists(path_file):
        os.remove(path_file)
    obj_position()
    rospy.spin()
