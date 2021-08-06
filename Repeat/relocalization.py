#!/usr/bin/env python
import numpy as np
from numpy import cos,sin,pi
import matplotlib.pyplot as plt
import csv 
import rospy
from sensor_msgs.msg import PointCloud,ChannelFloat32
from geometry_msgs.msg import PoseStamped,Point32,PoseArray,Pose
from scipy.spatial.transform import Rotation as R
import time
import os
from std_srvs.srv import Trigger,TriggerRequest
import copy    

class repeating():
    def __init__(self,map1,camera1):
        self.teach_cams = camera1
        self.map1 = None
        self.camera1 = None
        self.all_objs = []
        self.K = np.array([[528.96002, 0, 620.22998],
                           [0, 528.66498, 368.64499],
                           [0, 0, 1]])
        self.map_rotation = np.array([[1  ,        0      ,        0     ],
                                      [0  ,cos(0*pi/180),-sin(0*pi/180)],
                                      [0  ,sin(0*pi/180),cos(0*pi/180)]])
        self.preprocessing(map1,camera1)
        self.L1 = self.map1
        self.L2 = None
        self.pos1 = self.camera1
        self.scale = 1
        self.T12 = np.array([[1,0,0,0],
                             [0,1,0,0],
                             [0,0,1,0],
                             [0,0,0,1]])
        self.prev_T12 = np.array([[1,0,0,0],
                                  [0,1,0,0],
                                  [0,0,1,0],
                                  [0,0,0,1]])
        self.init_mode = True
        self.prev_scale  = 1
        self.prev_rot = np.array([[1,0,0],
                                  [0,1,0],
                                  [0,0,1]])

        rospy.init_node('listener', anonymous=True)
        self.cam_sub = rospy.Subscriber('/orb_pose', PoseStamped, self.get_pose)
        self.map_sub = rospy.Subscriber('/map_objects', PointCloud , self.get_maps)
        self.new_obj_pub = rospy.Publisher('/new_map_objects', PointCloud , queue_size=1)
        self.new_cam_pub = rospy.Publisher('/new_orb_pose', PoseStamped , queue_size=1)
        self.teach_obj_pub = rospy.Publisher('/teach_map_objects', PointCloud , queue_size=1 )
        self.teach_cam_pub = rospy.Publisher('/teach_orb_pose', PoseArray , queue_size=1)
        self.trig = rospy.ServiceProxy('/new_loc', Trigger)

    def trigger(self):
        res = Trigger()
        res.success = True
        return TriggerResponse(res) 

    def publish_teach(self):
        
	 #### publish new objs
         teach_cloud = PointCloud();
         teach_classes = ChannelFloat32()
         geo_points = []
         obj_classes = []
         for j in range(len(self.L1)):
             obj = self.L1[j]
             pt = Point32()
             pt.x = obj[0]
             pt.y = obj[1]
             pt.z = obj[2]
             geo_points.append(pt)
             obj_classes.append(obj[3])

         teach_classes.values = np.array(obj_classes)
         teach_classes.name = "teach objects in map"
         teach_cloud.header.frame_id = "map"
         teach_cloud.header.stamp = rospy.Time.now()
         teach_cloud.points = geo_points
         teach_cloud.channels = [teach_classes]
         self.teach_obj_pub.publish(teach_cloud)


 	#### publish teach camera frames
         self.short_cam1 = []
         for i,line in enumerate(self.teach_cams):
             if i%10 == 0:
                 self.short_cam1.append(line)
 
         teach_cam_arr = PoseArray()
         teach_cam_arr.header.stamp = rospy.Time.now()
         teach_cam_arr.header.frame_id = "map"

         tot_poses = []
         for i in range(len(self.short_cam1)):

             cur_pose = Pose()
             line = self.short_cam1[i]
             cur_pose.position.x = line[0]
             cur_pose.position.y = line[1]
             cur_pose.position.z = line[2]

             cur_pose.orientation.x = line[3]
             cur_pose.orientation.y = line[4]
             cur_pose.orientation.z = line[5]
             cur_pose.orientation.w = line[6]
             tot_poses.append(cur_pose)
         teach_cam_arr.poses = tot_poses
         self.teach_cam_pub.publish(teach_cam_arr)


    def preprocessing(self,map1,camera1):
        for i in range(len(map1)):
            obj = map1[i]
            obj_pos = np.array([float(obj[0]),float(obj[1]),float(obj[2])]).T
            obj_rotated_pos = np.dot(self.map_rotation,obj_pos)
            map1[i][0] = obj_rotated_pos[0]
            map1[i][1] = obj_rotated_pos[1]
            map1[i][2] = obj_rotated_pos[2]
        self.map1 = map1
        
        pos1 = []
        for i, line in enumerate(camera1):
            x1 = float(line[0])
            y1 = float(line[1])
            z1 = float(line[2])
            pos = np.array([x1,y1,z1]).T
            rotated_pos = np.dot(self.map_rotation,pos)
            pos1.append(rotated_pos)


        pos1 = np.array(pos1)
        self.camera1 = pos1


    def max_dis(self):
         dmax1 = 0
         for m1 in self.L1:
             for m2 in self.L1:
                 d = np.linalg.norm([m1[0]-m2[0],m1[2]-m2[2]])
                 if d>dmax1:
                     dmax1 = d

         dmax2 = 0
         for m1 in self.L2:
             for m2 in self.L2:
                 d = np.linalg.norm([m1[0]-m2[0],m1[2]-m2[2]])
                 if d>dmax2:
                     dmax2 = d

         max_dis_L1 = dmax1 
         max_dis_L2 = dmax2 
         return [max_dis_L1,max_dis_L2]
     
        
    def get_maps(self,obj_map):
        all_objs = []
        map_points = obj_map.points
        if len(map_points)<2:
            print("too soon")
            return 0
        for i in range(len(map_points)):
            classes = obj_map.channels[0].values
            cur_obj = classes[i]
            pt = Point32()
            pt = map_points[i]
            mpt = np.array([pt.x,pt.y,pt.z]).T
            new_mpt = np.dot(self.map_rotation,mpt)
            all_objs.append([new_mpt[0],new_mpt[1],new_mpt[2],cur_obj])
        self.all_objs = all_objs
        self.L2 = all_objs
        if self.T12[0][0] == 1:
            self.reloc()
       

    def get_pose(self,cam):
         self.publish_teach()

         c=np.zeros(3)
         R=np.zeros(4)
         cam_position = np.array([cam.pose.position.x,cam.pose.position.y,cam.pose.position.z]).T
         new_cam_position = np.dot(self.map_rotation,cam_position)
         
         c[0] = new_cam_position[0]
         c[1] = new_cam_position[1]
         c[2] = new_cam_position[2]

         R[0] = cam.pose.orientation.x
         R[1] = cam.pose.orientation.y
         R[2] = cam.pose.orientation.z
         R[3] = cam.pose.orientation.w

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
         cur_cam = cam.pose.position
         cur_cam_position = c

         if c[0] !=0 and c[1] !=0:
             with open('/home/mohammad/Desktop/cur_cam.csv', mode='a') as proj_file:
                 proj_writer = csv.writer(proj_file, delimiter=',')
                 proj_writer.writerow([c[0],c[1],c[2],R[0],R[1],R[2],R[3]])


         if c[0] !=0 and c[1] !=0 and self.L2 != None:
             self.transfer_motions(cur_cam_position,Proj)
         print(self.T12)
         print(self.scale)
         
    def theta(self,o1,o2,L,delta):
         
         p1 = np.array([[o1[0].tolist()],[o1[2].tolist()],[0],[1]])
         p2 = np.array([[o2[0].tolist()],[o2[2].tolist()],[0],[1]])
         ang = np.arctan2(p2[1]-p1[1],p2[0]-p1[0]) + delta * np.pi/180. 

         dp = np.array([p2[0]-p1[0],p2[1]-p1[1],1])
         Rot = np.array([[cos(-(ang-pi/2)),-sin(-(ang-pi/2)),0],
                         [sin(-(ang-pi/2)), cos(-(ang-pi/2)),0],
                         [       0        ,         0       ,1]])
         new_p1 = np.array([0,0]).T
         new_p2 = np.dot(Rot,dp).T
         new_p2 = new_p2[0:2]
         New_Map=[]
         New_Map.append([new_p1[0],new_p1[1],o1[3]])
         New_Map.append([new_p2[0],new_p2[1],o2[3]])
         for k in range(len(L)):
             if L[k,0] == o1[0] and L[k,2] == o1[2]:
                 continue
             if L[k,0] == o2[0] and L[k,2] == o2[2]:
                 continue
             dP2 = np.array([[L[k,0]-p1[0]],[L[k,2]-p1[1]],[1]])
             new_P = np.dot(Rot,dP2).T
             New_Map.append([new_P[0,0],new_P[0,1],L[k,3].tolist()])

         return New_Map
         
    def get_translation(self,obj1,obj2,scale,d_teta):
         
         for i in range(len(self.L1)):
             map_obj = self.L1[i]
             if map_obj[3] == obj1:
                 obj1_L1_loc = self.L1[i][:]
             if map_obj[3] == obj2:
                 obj2_L1_loc = self.L1[i][:]
         for i in range(len(self.L2)):
             map_obj = self.L2[i]
             if map_obj[3] == obj1:
                 obj1_L2_loc = self.L2[i][:]
             if map_obj[3] == obj2:
                 obj2_L2_loc = self.L2[i][:]

         ## finding To1:
         p1 = np.array([obj1_L1_loc[0],obj1_L1_loc[2],0,1]).T
         p2 = np.array([obj2_L1_loc[0],obj2_L1_loc[2],0,1]).T
         ang = np.arctan2(p2[1]-p1[1],p2[0]-p1[0])
         

         To1 = np.array([[cos((ang-pi/2)),-sin((ang-pi/2)), 0,  p1[0]],
                        [ sin((ang-pi/2)), cos((ang-pi/2)), 0,  p1[1]],
                        [    0           ,     0           ,1,     0 ],
                        [    0           ,     0           ,0,     1 ]])
         
         ## finding To2:
         p1 = np.array([obj1_L2_loc[0],obj1_L2_loc[2],0,1]).T
         p2 = np.array([obj2_L2_loc[0],obj2_L2_loc[2],0,1]).T
         ang = np.arctan2(p2[1]-p1[1],p2[0]-p1[0] )
         To2 = np.array([[cos((ang-pi/2)),-sin((ang-pi/2)) ,0,  p1[0]],
                        [ sin((ang-pi/2)), cos((ang-pi/2)) ,0,  p1[1]],
                        [    0           ,     0           ,1,     0 ],
                        [    0           ,     0           ,0,     1 ]])
               
         To2[0,3] = To2[0,3] * scale
         To2[1,3] = To2[1,3] * scale
         To2[2,3] = To2[2,3] * scale
         
         inv_To2 = np.linalg.inv(To2)
         T12 = np.dot(To1,inv_To2)
         
         return T12
         
    def transfer_motions(self,cur_cam,Proj):
         #### publish repeat camera frames
         cam_rot = Proj[0:3,0:3]          
         x2 = float(cur_cam[0]) *  self.scale
         y2 = float(cur_cam[1]) *  self.scale
         z2 = float(cur_cam[2]) *  self.scale

         new_T12 = np.array([[self.T12[0,0],0,-self.T12[0,1]],
                             [      0,     1       ,     0  ],
                             [-self.T12[1,0],0,self.T12[1,1]]])

         pos = np.array([x2,z2,0,1]).T
         new_position = np.dot(self.T12,pos)
         new_rot = np.dot(new_T12,cam_rot)
         print(cam_rot)
         print(new_rot)
         print() 
         r = R.from_matrix(new_rot)
         new_quat = r.as_quat()
         new_euler = r.as_euler('XZY') * 180/np.pi
         print("new_euler is:\n"+str(new_euler))
        
         r2 = R.from_matrix(cam_rot) 
         old_quat = r2.as_quat()
         old_euler = r2.as_euler('XZY')* 180/np.pi

         print("old_euler is:\n"+str(old_euler))  
         print()       


         if new_position[0] !=0 and new_position[1] != 0:
             with open('/home/mohammad/Desktop/new_cam.csv', mode='a') as proj_file:
                 proj_writer = csv.writer(proj_file, delimiter=',')
                 proj_writer.writerow([new_position[0],y2,new_position[1],new_quat[0],new_quat[1],new_quat[2],new_quat[3]])

         new_cam = PoseStamped()
         new_cam.header.stamp = rospy.Time.now()
         new_cam.header.frame_id = "map"
         new_cam.pose.position.x = new_position[0]
         new_cam.pose.position.y = y2
         new_cam.pose.position.z = new_position[1]
         
         new_cam.pose.orientation.x = new_quat[0]
         new_cam.pose.orientation.y = new_quat[1]
         new_cam.pose.orientation.z = new_quat[2]
         new_cam.pose.orientation.w = -new_quat[3]  # was -

         self.new_cam_pub.publish(new_cam)

         #### publish new objs
         cloud = PointCloud();
         classes = ChannelFloat32()
         geo_points = []
         obj_classes = []
         for j in range(len(self.L2)):
             obj = self.L2[j]
             obj_pos = np.array([self.scale*obj[0], self.scale*obj[2],0,1]).T
             res = np.dot(self.T12 , obj_pos)
             pt = Point32()
             pt.x = res[0]
             pt.y = self.scale*obj[1]
             pt.z = res[1]
             geo_points.append(pt)
             obj_classes.append(obj[3])

         classes.values = np.array(obj_classes)
         classes.name = "modified objects in map"
         cloud.header.frame_id = "map"
         cloud.header.stamp = rospy.Time.now()
         cloud.points = geo_points
         cloud.channels = [classes]
         self.new_obj_pub.publish(cloud)


    def hungarian(self,L1_main,L2_main):
         res = []
         [max_dis_L1,max_dis_L2]=self.max_dis()
         dteta = np.array([0])
         for delta in dteta:
             L1 = np.array(L1_main)
             L2 = np.array(L2_main)
             
             for i in range(len(L2)-1,-1,-1):
                 cur_obj = L2[i]
                 found = 0
                 for j in range(len(L1)-1,-1,-1):
                     prev_obj = L1[j]
                     if prev_obj[3] == cur_obj[3]:
                         found = 1
                 if found == 0:
                     L2 = np.delete(L2,i,0)
    
             for i in range(len(L1)-1,-1,-1):
                 cur_obj = L1[i]
                 found = 0
                 for j in range(len(L2)-1,-1,-1):
                     prev_obj = L2[j]
                     if prev_obj[3] == cur_obj[3]:
                         found = 1
                 if found == 0:
                     L1 = np.delete(L1,i,0)
                 
             L1_maps = []
             for i in range(len(L1)):
                 for j in range(len(L1)):
                     if i == j:
                         continue
                     
                     dis = np.linalg.norm([L1[i,0]-L1[j,0],L1[i,2]-L1[j,2]])
                     if max_dis_L1/dis>5:
                         continue
                     
                     New_Map = self.theta(L1[i,:],L1[j,:],L1,0)             
                     L1_maps.append(New_Map)
                     
             L2_maps = []                 
             for i in range(len(L2)):
                 for j in range(len(L2)):
                     if i == j:
                         continue
                     
                     dis = np.linalg.norm([L2[i,0]-L2[j,0],L2[i,2]-L2[j,2]])
                     if max_dis_L2/dis>5:
                         continue
                     
                     New_Map = self.theta(L2[i,:],L2[j,:],L2,delta)             
                     L2_maps.append(New_Map)
                
             
             for i in range(len(L2_maps)):
                 cur_frame = L2_maps[i]
                 cur_orig = cur_frame[0]
                 cur_aim = cur_frame[1]
                 cur_others = np.array(cur_frame[2:])
                 for j in range(len(L1_maps)):
                     prev_frame = L1_maps[j]
                     prev_orig = prev_frame[0]
                     prev_aim = prev_frame[1]
                     prev_others = np.array(prev_frame[2:])
                     
                     if cur_orig[2] != prev_orig[2] or cur_aim[2] != prev_aim[2]:
                         continue
                     
                     # scaling
                     map_ratio = prev_aim[1]/cur_aim[1]
                     s_range = map_ratio
                     for S in s_range:
                         er_list = []
                         error = np.linalg.norm(np.array([prev_aim[0] - S*cur_aim[0] , prev_aim[1] - S*cur_aim[1]]))
                         for k in range(len(prev_others)):
                             for m in range(len(cur_others)):
                                 if prev_others[k,2] != cur_others[m,2]:
                                     continue
                                 dis = np.array([prev_others[k,0] - S*cur_others[m,0] , prev_others[k,1] - S*cur_others[m,1]])
                                 er = np.linalg.norm(dis)
                                 er_list.append(er)
                         if len(er_list) != 0:
                             sorted_er = sorted(er_list)
                             if len(L2)/2. % 1 == 0:
                                 er_objs = len(L2)/2. -1 
                             else:
                                 er_objs = np.floor(len(L2)/2.)
                                 
                             chosen_er = sorted_er[0:int(er_objs)]    
                             if len(chosen_er)>1:
                                 chosen_er = sum(chosen_er)        
                             error = error + chosen_er

                         res.append([i,S,delta,error])               
                             
         res = np.array(res)
         sort = res[res[:,3].argsort()]
         scale = sort[0,1]
         d_teta = sort[0,2]
         best_map_id = int(sort[0,0])
         best_map = L2_maps[best_map_id]
         obj1 = best_map[0][2]
         obj2 = best_map[1][2]
         T12 = self.get_translation(obj1,obj2,scale,d_teta) 
                   
         return [T12,scale]         

    def reloc(self):
        
        [self.T12,self.scale] = self.hungarian(self.L1,self.L2)
        print(self.T12)
        print(self.scale) 
        if self.scale != self.prev_scale:
             print("new scale")

        print(self.scale)
        print(self.prev_scale)
        print()
        self.prev_scale = copy.deepcopy(self.scale)

        


if __name__ == '__main__':

    cur_cam_file = '/home/mohammad/Desktop/cur_cam.csv'
    if os.path.exists(cur_cam_file):
        os.remove(cur_cam_file)

    new_cam_file = '/home/mohammad/Desktop/new_cam.csv'
    if os.path.exists(new_cam_file):
        os.remove(new_cam_file)

    new_cam_file = '/home/mohammad/Desktop/init_path.csv'
    if os.path.exists(new_cam_file):
        os.remove(new_cam_file)


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
    repeating(teach_map, camera1)
    rospy.spin()


