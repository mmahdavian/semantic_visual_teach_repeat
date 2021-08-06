#include <iostream>
#include "ros/ros.h"
#include <vector>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <opencv2/core/core.hpp>
//#include </home/mohammad/Mohammad_ws/ORB-SLAM/ORB_SLAM3/include/Converter.h>
#include <Eigen/Dense>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;

class map_show{

private:
     ros::NodeHandle n;
     ros::Subscriber Mo_sub;
     ros::Subscriber Kf_sub;
     ros::Subscriber NMo_sub;
     ros::Subscriber NKf_sub;
     ros::Subscriber NKf_nd_sub;
     ros::Subscriber teach_Mo_sub;
     ros::Subscriber teach_Kf_sub;
     ros::Subscriber goal_points_sub;
//     ros::Publisher marker_pub_cam;
     ros::Publisher marker_pub_obj;
     ros::Publisher marker_pub_cam_arr;
     ros::Publisher new_marker_pub_obj;
     ros::Publisher new_marker_pub_cam_arr;
     ros::Publisher new_marker_pub_cam_arr_nd;
     ros::Publisher teach_marker_pub_obj;
     ros::Publisher teach_marker_pub_cam_arr;
     ros::Publisher goal_points_pub;

public:
     std::vector<geometry_msgs::Point32> all_objects;     
     std::vector<geometry_msgs::Point32> new_all_objects;   
     std::vector<geometry_msgs::Point32> teach_all_objects;
     std::vector<geometry_msgs::Point32> all_goal_points;
     
     float x_obj;
     float y_obj;
     float z_obj;
     float x_cam;
     float y_cam;
     float z_cam;
     float R[4];

     float nx_obj;
     float ny_obj;
     float nz_obj;
     float nx_cam;
     float ny_cam;
     float nz_cam;
     float NR[4];

     float nx_nd_cam;
     float ny_nd_cam;
     float nz_nd_cam;
     float NR_nd[4];

     float tx_obj;
     float ty_obj;
     float tz_obj;
     float tx_cam;
     float ty_cam;
     float tz_cam;
     float TR[4];

     float x_gp;
     float y_gp;
     float z_gp;

     std::vector<visualization_msgs::Marker> all_cam_markers;
     std::vector<visualization_msgs::Marker> new_all_cam_markers;
     std::vector<visualization_msgs::Marker> new_all_cam_markers_nodrift;
     std::vector<visualization_msgs::Marker> teach_cam_markers;

  //   std::vector<visualization_msgs::Marker> all_markers;
     visualization_msgs::MarkerArray Markerarr;
     visualization_msgs::MarkerArray Posesarr;
     visualization_msgs::MarkerArray New_Markerarr;
     visualization_msgs::MarkerArray New_Posesarr;
     visualization_msgs::MarkerArray New_Posesarr_nd;
     visualization_msgs::MarkerArray teach_Markerarr;
     visualization_msgs::MarkerArray teach_Posesarr;
     visualization_msgs::MarkerArray goal_pointsarr;
     

     geometry_msgs::PoseStamped pose;
     geometry_msgs::PoseStamped new_pose;
     geometry_msgs::PoseStamped teach_pose;
            
     float init_height = 1;
 
map_show():
n("~"){
     
    
     Mo_sub = n.subscribe("/map_objects",1,&map_show::Objects , this);
     Kf_sub = n.subscribe("/orb_pose",1,&map_show::Camera_poses , this);
     marker_pub_obj = n.advertise<visualization_msgs::MarkerArray>("objects_marker_array", 1);
//     marker_pub_cam = n.advertise<visualization_msgs::Marker>("camera_pose_marker", 1);
     NMo_sub = n.subscribe("/new_map_objects",1,&map_show::New_Objects , this);
     NKf_sub = n.subscribe("/new_orb_pose",1,&map_show::New_Camera_poses , this);
     NKf_nd_sub = n.subscribe("/orb_pose_drifted",1,&map_show::New_Camera_poses_drifted , this);

     teach_Mo_sub = n.subscribe("/teach_map_objects",1,&map_show::teach_Objects , this);
     teach_Kf_sub = n.subscribe("/teach_orb_pose",1,&map_show::teach_Camera_poses , this);          

     goal_points_sub = n.subscribe("/goal_points_pose",1,&map_show::goal_points , this);

     teach_marker_pub_obj = n.advertise<visualization_msgs::MarkerArray>("teach_objects_marker_array", 1);
     teach_marker_pub_cam_arr = n.advertise<visualization_msgs::MarkerArray>("teach_camera_marker_array", 1);

     goal_points_pub = n.advertise<visualization_msgs::MarkerArray>("goal_points_marker_array", 1);
     marker_pub_cam_arr = n.advertise<visualization_msgs::MarkerArray>("camera_pose_marker_array", 1);
     new_marker_pub_obj = n.advertise<visualization_msgs::MarkerArray>("new_objects_marker_array", 1);
//     marker_pub_cam = n.advertise<visualization_msgs::Marker>("camera_pose_marker", 1);
     new_marker_pub_cam_arr = n.advertise<visualization_msgs::MarkerArray>("new_camera_pose_marker_array", 1);
     new_marker_pub_cam_arr_nd = n.advertise<visualization_msgs::MarkerArray>("camera_pose_marker_array_drifted", 1);


}
~map_show(){}

void Objects(const sensor_msgs::PointCloud::ConstPtr& msg1){        
     all_objects = msg1->points;
     Markerarr.markers.resize(all_objects.size());
     for(size_t i=0 ; i<all_objects.size(); i++){
          geometry_msgs::Point32 obj;
          obj = all_objects[i];
          x_obj = obj.x;
          y_obj = obj.y;
          z_obj = obj.z;

          float orb_to_ros[3][3] = {{0,0,1},
                                    {-1,0,0},
                                    {0,-1,0}};
         
          cv::Mat ORB_ROS = cv::Mat(3, 3, CV_32FC1, &orb_to_ros);
          float object_point[3][1] = {{x_obj},{y_obj},{z_obj}};
          cv::Mat OBJ_Point = cv::Mat(3, 1, CV_32FC1, &object_point); 

          cv::Mat OBJ_transformed = ORB_ROS * OBJ_Point;
          float x_obj_tr = OBJ_transformed.at<float>(0,0);
          float y_obj_tr = OBJ_transformed.at<float>(1,0);
          float z_obj_tr = OBJ_transformed.at<float>(2,0);

          Markerarr.markers[i].header.stamp = ros::Time();
          Markerarr.markers[i].header.frame_id = "world";
          Markerarr.markers[i].ns = "object";
          Markerarr.markers[i].id = i;
          Markerarr.markers[i].type = visualization_msgs::Marker::SPHERE;
          Markerarr.markers[i].action = visualization_msgs::Marker::ADD;
          Markerarr.markers[i].pose.position.x = z_obj;
          Markerarr.markers[i].pose.position.y = -x_obj;
          Markerarr.markers[i].pose.position.z = -y_obj+init_height;
          Markerarr.markers[i].pose.orientation.x = 0;
          Markerarr.markers[i].pose.orientation.y = 0;
          Markerarr.markers[i].pose.orientation.z = 0;
          Markerarr.markers[i].pose.orientation.w = 0;
          Markerarr.markers[i].scale.x = 0.1;
          Markerarr.markers[i].scale.y = 0.1;
          Markerarr.markers[i].scale.z = 0.1;
          Markerarr.markers[i].color.r = 1.0f;
          Markerarr.markers[i].color.g = 0.0f;
          Markerarr.markers[i].color.b = 0.0f;
          Markerarr.markers[i].color.a = 1.0f;
          Markerarr.markers[i].lifetime = ros::Duration();
     }
     marker_pub_obj.publish(Markerarr);
}

void New_Objects(const sensor_msgs::PointCloud::ConstPtr& msg3){        
     new_all_objects = msg3->points;
     New_Markerarr.markers.resize(new_all_objects.size());
     for(size_t i=0 ; i<new_all_objects.size(); i++){
          geometry_msgs::Point32 obj;
          obj = new_all_objects[i];
          nx_obj = obj.x;
          ny_obj = obj.y;
          nz_obj = obj.z;

          float orb_to_ros[3][3] = {{0,0,1},
                                    {-1,0,0},
                                    {0,-1,0}};
         
          cv::Mat ORB_ROS = cv::Mat(3, 3, CV_32FC1, &orb_to_ros);
          float object_point[3][1] = {{nx_obj},{ny_obj},{nz_obj}};
          cv::Mat OBJ_Point = cv::Mat(3, 1, CV_32FC1, &object_point); 

          cv::Mat OBJ_transformed = ORB_ROS * OBJ_Point;
          float nx_obj_tr = OBJ_transformed.at<float>(0,0);
          float ny_obj_tr = OBJ_transformed.at<float>(1,0);
          float nz_obj_tr = OBJ_transformed.at<float>(2,0);

          New_Markerarr.markers[i].header.stamp = ros::Time();
          New_Markerarr.markers[i].header.frame_id = "world";
          New_Markerarr.markers[i].ns = "object";
          New_Markerarr.markers[i].id = i;
          New_Markerarr.markers[i].type = visualization_msgs::Marker::CUBE;
          New_Markerarr.markers[i].action = visualization_msgs::Marker::ADD;
          New_Markerarr.markers[i].pose.position.x = nz_obj;
          New_Markerarr.markers[i].pose.position.y = -nx_obj;
          New_Markerarr.markers[i].pose.position.z = -ny_obj+init_height;
          New_Markerarr.markers[i].pose.orientation.x = 0;
          New_Markerarr.markers[i].pose.orientation.y = 0;
          New_Markerarr.markers[i].pose.orientation.z = 0;
          New_Markerarr.markers[i].pose.orientation.w = 0;
          New_Markerarr.markers[i].scale.x = 0.1;
          New_Markerarr.markers[i].scale.y = 0.1;
          New_Markerarr.markers[i].scale.z = 0.1;
          New_Markerarr.markers[i].color.r = 0.0f;
          New_Markerarr.markers[i].color.g = 0.0f;
          New_Markerarr.markers[i].color.b = 1.0f;
          New_Markerarr.markers[i].color.a = 1.0f;
          New_Markerarr.markers[i].lifetime = ros::Duration();
     }
     new_marker_pub_obj.publish(New_Markerarr);
}


void teach_Objects(const sensor_msgs::PointCloud::ConstPtr& msg5){        
     teach_all_objects = msg5->points;
     teach_Markerarr.markers.resize(teach_all_objects.size());
     for(size_t i=0 ; i<teach_all_objects.size(); i++){
          geometry_msgs::Point32 obj;
          obj = teach_all_objects[i];
          tx_obj = obj.x;
          ty_obj = obj.y;
          tz_obj = obj.z;

          float orb_to_ros[3][3] = {{0,0,1},
                                    {-1,0,0},
                                    {0,-1,0}};
         
          cv::Mat ORB_ROS = cv::Mat(3, 3, CV_32FC1, &orb_to_ros);
          float object_point[3][1] = {{tx_obj},{ty_obj},{tz_obj}};
          cv::Mat OBJ_Point = cv::Mat(3, 1, CV_32FC1, &object_point); 

          cv::Mat OBJ_transformed = ORB_ROS * OBJ_Point;
          float tx_obj_tr = OBJ_transformed.at<float>(0,0);
          float ty_obj_tr = OBJ_transformed.at<float>(1,0);
          float tz_obj_tr = OBJ_transformed.at<float>(2,0);

          teach_Markerarr.markers[i].header.stamp = ros::Time();
          teach_Markerarr.markers[i].header.frame_id = "world";
          teach_Markerarr.markers[i].ns = "object";
          teach_Markerarr.markers[i].id = i;
          teach_Markerarr.markers[i].type = visualization_msgs::Marker::CUBE;
          teach_Markerarr.markers[i].action = visualization_msgs::Marker::ADD;
          teach_Markerarr.markers[i].pose.position.x = tz_obj;
          teach_Markerarr.markers[i].pose.position.y = -tx_obj;
          teach_Markerarr.markers[i].pose.position.z = -ty_obj+init_height;
          teach_Markerarr.markers[i].pose.orientation.x = 0;
          teach_Markerarr.markers[i].pose.orientation.y = 0;
          teach_Markerarr.markers[i].pose.orientation.z = 0;
          teach_Markerarr.markers[i].pose.orientation.w = 0;
          teach_Markerarr.markers[i].scale.x = 0.1;
          teach_Markerarr.markers[i].scale.y = 0.1;
          teach_Markerarr.markers[i].scale.z = 0.1;
          teach_Markerarr.markers[i].color.r = 1.0f;
          teach_Markerarr.markers[i].color.g = 0.0f;
          teach_Markerarr.markers[i].color.b = 0.0f;
          teach_Markerarr.markers[i].color.a = 1.0f;
          teach_Markerarr.markers[i].lifetime = ros::Duration();
          
     }
     cout<<"hiii"<<endl;
     teach_marker_pub_obj.publish(teach_Markerarr);
}

void goal_points(const sensor_msgs::PointCloud::ConstPtr& msg7){        
     all_goal_points = msg7->points;
     goal_pointsarr.markers.resize(all_goal_points.size());
     for(size_t i=0 ; i<all_goal_points.size(); i++){
          geometry_msgs::Point32 obj;
          obj = all_goal_points[i];
          x_gp = obj.x;
          y_gp = obj.y;
          z_gp = obj.z;

          cout<<x_gp<<" "<<y_gp<<" "<<z_gp<<endl;

          float orb_to_ros[3][3] = {{0,0,1},
                                    {-1,0,0},
                                    {0,-1,0}};
         
          cv::Mat ORB_ROS = cv::Mat(3, 3, CV_32FC1, &orb_to_ros);
          float object_point[3][1] = {{x_gp},{y_gp},{z_gp}};
          cv::Mat OBJ_Point = cv::Mat(3, 1, CV_32FC1, &object_point); 

          cv::Mat OBJ_transformed = ORB_ROS * OBJ_Point;
          float tx_obj_tr = OBJ_transformed.at<float>(0,0);
          float ty_obj_tr = OBJ_transformed.at<float>(1,0);
          float tz_obj_tr = OBJ_transformed.at<float>(2,0);

          goal_pointsarr.markers[i].header.stamp = ros::Time();
          goal_pointsarr.markers[i].header.frame_id = "world";
          goal_pointsarr.markers[i].ns = "object";
          goal_pointsarr.markers[i].id = i;
          goal_pointsarr.markers[i].type = visualization_msgs::Marker::SPHERE;
          goal_pointsarr.markers[i].action = visualization_msgs::Marker::ADD;
          goal_pointsarr.markers[i].pose.position.x = z_gp;
          goal_pointsarr.markers[i].pose.position.y = -x_gp;
          goal_pointsarr.markers[i].pose.position.z = -y_gp+init_height;
          goal_pointsarr.markers[i].pose.orientation.x = 0;
          goal_pointsarr.markers[i].pose.orientation.y = 0;
          goal_pointsarr.markers[i].pose.orientation.z = 0;
          goal_pointsarr.markers[i].pose.orientation.w = 0;
          goal_pointsarr.markers[i].scale.x = 0.1;
          goal_pointsarr.markers[i].scale.y = 0.1;
          goal_pointsarr.markers[i].scale.z = 0.1;
          goal_pointsarr.markers[i].color.r = 1.0f;
          goal_pointsarr.markers[i].color.g = 0.0f;
          goal_pointsarr.markers[i].color.b = 1.0f;
          goal_pointsarr.markers[i].color.a = 1.0f;
          goal_pointsarr.markers[i].lifetime = ros::Duration();
     }
     goal_points_pub.publish(goal_pointsarr);
}

void Camera_poses(const geometry_msgs::PoseStamped::ConstPtr& msg2){
     x_cam = msg2->pose.position.x;
     y_cam = msg2->pose.position.y;
     z_cam = msg2->pose.position.z;
//     cout<<"cur cams are: "<<x_cam<<" "<<y_cam<<" "<<z_cam<<endl;

     R[0] = msg2->pose.orientation.x;
     R[1] = msg2->pose.orientation.y;
     R[2] = msg2->pose.orientation.z;
     R[3] = msg2->pose.orientation.w;

//     cout<<"cams poses are: "<<R[0]<<" "<<R[1]<<" "<<R[2]<<" "<<R[3]<<endl;

     float Rot[3][3] = {{1-2*(R[1]*R[1])-2*(R[2]*R[2]),2*R[0]*R[1]+2*R[2]*R[3],2*R[0]*R[2]-2*R[1]*R[3]},
                        {2*R[0]*R[1]-2*R[2]*R[3],1-2*(R[0]*R[0])-2*(R[2]*R[2]),2*R[1]*R[2]+2*R[0]*R[3]},
                        {2*R[0]*R[2]+2*R[1]*R[3],2*R[1]*R[2]-2*R[0]*R[3],1-2*(R[0]*R[0])-2*(R[1]*R[1])}};


     cv::Mat Rotation = cv::Mat(3, 3, CV_32FC1, &Rot);
     //cout<<"Rotation is: "<<Rotation<<endl;

     float tcw[3][1] = {{-x_cam},{-y_cam},{-z_cam}};
     cv::Mat tcw_ros = cv::Mat(3, 1, CV_32FC1, &tcw);

     cv::Mat tcw_orb = Rotation*tcw_ros;

     float proj[3][4];

     proj[0][0] = Rot[0][0];
     proj[0][1] = Rot[0][1];
     proj[0][2] = Rot[0][2];
     proj[1][0] = Rot[1][0];
     proj[1][1] = Rot[1][1];
     proj[1][2] = Rot[1][2];
     proj[2][0] = Rot[2][0];
     proj[2][1] = Rot[2][1];
     proj[2][2] = Rot[2][2];
     proj[0][3] = tcw_orb.at<float> (0,0);
     proj[1][3] = tcw_orb.at<float> (1,0);
     proj[2][3] = tcw_orb.at<float> (2,0);

     cv::Mat Proj = cv::Mat(3, 4, CV_32FC1, &proj);   //same as ORB_slam
     cv::Mat transform = TransformFromMat(Proj);    //gets transformed to rviz format
     cv::Mat Rwc = transform.rowRange(0,3).colRange(0,3);
     cv::Mat twc = transform.rowRange(0,3).col(3);
     vector<float> q = toQuaternion(Rwc);

     tf::Transform new_transform;
     new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));

     tf::Quaternion quaternion(q[0], q[1], q[2], q[3]);
     new_transform.setRotation(quaternion);
     tf::poseTFToMsg(new_transform, pose.pose);

     if (twc.at<float>(0, 0) != 0){
     save_poses(twc.at<float>(0, 0),twc.at<float>(1,0),twc.at<float>(2,0),q);
     }
     visualize_cam();
     
}


void teach_Camera_poses(const geometry_msgs::PoseArray::ConstPtr& msg6){
     for(size_t i = 0;i<msg6->poses.size();i++){
     tx_cam = msg6->poses[i].position.x;
     ty_cam = msg6->poses[i].position.y;
     tz_cam = msg6->poses[i].position.z;

     TR[0] = msg6->poses[i].orientation.x;
     TR[1] = msg6->poses[i].orientation.y;
     TR[2] = msg6->poses[i].orientation.z;
     TR[3] = msg6->poses[i].orientation.w;

     float Rot[3][3] = {{1-2*(TR[1]*TR[1])-2*(TR[2]*TR[2]),2*TR[0]*TR[1]+2*TR[2]*TR[3],2*TR[0]*TR[2]-2*TR[1]*TR[3]},
                        {2*TR[0]*TR[1]-2*TR[2]*TR[3],1-2*(TR[0]*TR[0])-2*(TR[2]*TR[2]),2*TR[1]*TR[2]+2*TR[0]*TR[3]},
                        {2*TR[0]*TR[2]+2*TR[1]*TR[3],2*TR[1]*TR[2]-2*TR[0]*TR[3],1-2*(TR[0]*TR[0])-2*(TR[1]*TR[1])}};

     cv::Mat Rotation = cv::Mat(3, 3, CV_32FC1, &Rot);
     //cout<<"Rotation is: "<<Rotation<<endl;

     float tcw[3][1] = {{-tx_cam},{-ty_cam},{-tz_cam}};
     cv::Mat tcw_ros = cv::Mat(3, 1, CV_32FC1, &tcw);

     cv::Mat tcw_orb = Rotation*tcw_ros;

     float proj[3][4];

     proj[0][0] = Rot[0][0];
     proj[0][1] = Rot[0][1];
     proj[0][2] = Rot[0][2];
     proj[1][0] = Rot[1][0];
     proj[1][1] = Rot[1][1];
     proj[1][2] = Rot[1][2];
     proj[2][0] = Rot[2][0];
     proj[2][1] = Rot[2][1];
     proj[2][2] = Rot[2][2];
     proj[0][3] = tcw_orb.at<float> (0,0);
     proj[1][3] = tcw_orb.at<float> (1,0);
     proj[2][3] = tcw_orb.at<float> (2,0);

     cv::Mat Proj = cv::Mat(3, 4, CV_32FC1, &proj);   //same as ORB_slam
     cv::Mat transform = TransformFromMat(Proj);    //gets transformed to rviz format
     cv::Mat Rwc = transform.rowRange(0,3).colRange(0,3);
     cv::Mat twc = transform.rowRange(0,3).col(3);
     vector<float> q = toQuaternion(Rwc);

     tf::Transform new_transform;
     new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));

     tf::Quaternion quaternion(q[0], q[1], q[2], q[3]);
     new_transform.setRotation(quaternion);
     tf::poseTFToMsg(new_transform, teach_pose.pose);

     if (twc.at<float>(0, 0) != 0){
     teach_save_poses(twc.at<float>(0, 0),twc.at<float>(1,0),twc.at<float>(2,0),q);
     }

     }

     teach_visualize_cam();
     
}


void New_Camera_poses(const geometry_msgs::PoseStamped::ConstPtr& msg4){
     nx_cam = msg4->pose.position.x;
     ny_cam = msg4->pose.position.y;
     nz_cam = msg4->pose.position.z;

//     cout<<"new cams are: "<<nx_cam<<" "<<ny_cam<<" "<<nz_cam<<endl;

     NR[0] = msg4->pose.orientation.x;
     NR[1] = msg4->pose.orientation.y;
     NR[2] = msg4->pose.orientation.z;
     NR[3] = msg4->pose.orientation.w;
//     cout<<"new cams poses are: "<<NR[0]<<" "<<NR[1]<<" "<<NR[2]<<" "<<NR[3]<<endl;

     float NRot[3][3] = {{1-2*(NR[1]*NR[1])-2*(NR[2]*NR[2]),2*NR[0]*NR[1]+2*NR[2]*NR[3],2*NR[0]*NR[2]-2*NR[1]*NR[3]},
                        {2*NR[0]*NR[1]-2*NR[2]*NR[3],1-2*(NR[0]*NR[0])-2*(NR[2]*NR[2]),2*NR[1]*NR[2]+2*NR[0]*NR[3]},
                        {2*NR[0]*NR[2]+2*NR[1]*NR[3],2*NR[1]*NR[2]-2*NR[0]*NR[3],1-2*(NR[0]*NR[0])-2*(NR[1]*NR[1])}};


     cv::Mat Rotation = cv::Mat(3, 3, CV_32FC1, &NRot);
     //cout<<"Rotation is: "<<Rotation<<endl;

     float tcw[3][1] = {{-nx_cam},{-ny_cam},{-nz_cam}};
     cv::Mat tcw_ros = cv::Mat(3, 1, CV_32FC1, &tcw);

     cv::Mat tcw_orb = Rotation*tcw_ros;

     float proj[3][4];

     proj[0][0] = NRot[0][0];
     proj[0][1] = NRot[0][1];
     proj[0][2] = NRot[0][2];
     proj[1][0] = NRot[1][0];
     proj[1][1] = NRot[1][1];
     proj[1][2] = NRot[1][2];
     proj[2][0] = NRot[2][0];
     proj[2][1] = NRot[2][1];
     proj[2][2] = NRot[2][2];
//     proj[0][3] = nx_cam;
//    proj[1][3] = ny_cam;
//     proj[2][3] = nz_cam;
     proj[0][3] = tcw_orb.at<float> (0,0);
     proj[1][3] = tcw_orb.at<float> (1,0);
     proj[2][3] = tcw_orb.at<float> (2,0);


     cv::Mat Proj = cv::Mat(3, 4, CV_32FC1, &proj);   //same as ORB_slam
     cv::Mat transform = TransformFromMat(Proj);    //gets transformed to rviz format
     cv::Mat Rwc = transform.rowRange(0,3).colRange(0,3);
     cv::Mat twc = transform.rowRange(0,3).col(3);
     vector<float> nq = toQuaternion(Rwc);

     tf::Transform new_transform;
     new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));

     tf::Quaternion quaternion(nq[0], nq[1], nq[2], nq[3]);
     new_transform.setRotation(quaternion);
     tf::poseTFToMsg(new_transform, new_pose.pose);

     if (twc.at<float>(0, 0) != 0){
     new_save_poses(twc.at<float>(0, 0),twc.at<float>(1,0),twc.at<float>(2,0),nq);
     }
     new_visualize_cam();
     
}

void New_Camera_poses_drifted(const geometry_msgs::PoseStamped::ConstPtr& msg8){
     nx_nd_cam = msg8->pose.position.x;
     ny_nd_cam = msg8->pose.position.y;
     nz_nd_cam = msg8->pose.position.z;

//     cout<<"new cams are: "<<nx_cam<<" "<<ny_cam<<" "<<nz_cam<<endl;

     NR_nd[0] = msg8->pose.orientation.x;
     NR_nd[1] = msg8->pose.orientation.y;
     NR_nd[2] = msg8->pose.orientation.z;
     NR_nd[3] = msg8->pose.orientation.w;
//     cout<<"new cams poses are: "<<NR[0]<<" "<<NR[1]<<" "<<NR[2]<<" "<<NR[3]<<endl;

     float NRot_nd[3][3] = {{1-2*(NR_nd[1]*NR_nd[1])-2*(NR_nd[2]*NR_nd[2]),2*NR_nd[0]*NR_nd[1]+2*NR_nd[2]*NR_nd[3],2*NR_nd[0]*NR_nd[2]-2*NR_nd[1]*NR_nd[3]},
                           {2*NR_nd[0]*NR_nd[1]-2*NR_nd[2]*NR_nd[3],1-2*(NR_nd[0]*NR_nd[0])-2*(NR_nd[2]*NR_nd[2]),2*NR_nd[1]*NR_nd[2]+2*NR_nd[0]*NR_nd[3]},
                           {2*NR_nd[0]*NR_nd[2]+2*NR_nd[1]*NR_nd[3],2*NR_nd[1]*NR_nd[2]-2*NR_nd[0]*NR_nd[3],1-2*(NR_nd[0]*NR_nd[0])-2*(NR_nd[1]*NR_nd[1])}};


     cv::Mat Rotation = cv::Mat(3, 3, CV_32FC1, &NRot_nd);
     //cout<<"Rotation is: "<<Rotation<<endl;

     float tcw[3][1] = {{-nx_nd_cam},{-ny_nd_cam},{-nz_nd_cam}};
     cv::Mat tcw_ros = cv::Mat(3, 1, CV_32FC1, &tcw);

     cv::Mat tcw_orb = Rotation*tcw_ros;

     float proj[3][4];

     proj[0][0] = NRot_nd[0][0];
     proj[0][1] = NRot_nd[0][1];
     proj[0][2] = NRot_nd[0][2];
     proj[1][0] = NRot_nd[1][0];
     proj[1][1] = NRot_nd[1][1];
     proj[1][2] = NRot_nd[1][2];
     proj[2][0] = NRot_nd[2][0];
     proj[2][1] = NRot_nd[2][1];
     proj[2][2] = NRot_nd[2][2];
     proj[0][3] = tcw_orb.at<float> (0,0);
     proj[1][3] = tcw_orb.at<float> (1,0);
     proj[2][3] = tcw_orb.at<float> (2,0);


     cv::Mat Proj = cv::Mat(3, 4, CV_32FC1, &proj);   //same as ORB_slam
     cv::Mat transform = TransformFromMat(Proj);    //gets transformed to rviz format
     cv::Mat Rwc = transform.rowRange(0,3).colRange(0,3);
     cv::Mat twc = transform.rowRange(0,3).col(3);
     vector<float> nq = toQuaternion(Rwc);

     tf::Transform new_transform;
     new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));

     tf::Quaternion quaternion(nq[0], nq[1], nq[2], nq[3]);
     new_transform.setRotation(quaternion);
     tf::poseTFToMsg(new_transform, new_pose.pose);

     if (twc.at<float>(0, 0) != 0){
     new_save_poses_nodrift(twc.at<float>(0, 0),twc.at<float>(1,0),twc.at<float>(2,0),nq);
     }
     new_visualize_cam_nodrift();     
}


void save_poses(float x_cam, float y_cam, float z_cam, vector<float> q){

     visualization_msgs::Marker marker_cam;
     marker_cam.header.stamp = ros::Time();
     marker_cam.header.frame_id = "world";
     marker_cam.ns = "camera";
     marker_cam.id = 1;
     marker_cam.type = visualization_msgs::Marker::ARROW;
     marker_cam.action = visualization_msgs::Marker::ADD;
    // cout<<x_cam<<" "<<y_cam<<" "<<z_cam<<endl;
     marker_cam.pose.position.x = x_cam;
     marker_cam.pose.position.y = y_cam;
     marker_cam.pose.position.z = z_cam+init_height;
     marker_cam.pose.orientation.x = q[0];
     marker_cam.pose.orientation.y = q[1];
     marker_cam.pose.orientation.z = q[2];
     marker_cam.pose.orientation.w = q[3];
     marker_cam.scale.x = 0.2;
     marker_cam.scale.y = 0.05;
     marker_cam.scale.z = 0.05;
     marker_cam.color.r = 0.0f;
     marker_cam.color.g = 1.0f;
     marker_cam.color.b = 0.0f;
     marker_cam.color.a = 1.0f;
     marker_cam.lifetime = ros::Duration();
     all_cam_markers.push_back(marker_cam);
//     if(all_cam_markers.size() > 1){
//	all_cam_markers.erase(all_cam_markers.begin());
//     }

     cout<<"size of cam poses are: "<<all_cam_markers.size()<<endl; 
}

void teach_save_poses(float x_cam, float y_cam, float z_cam, vector<float> q){

     visualization_msgs::Marker marker_cam;
     marker_cam.header.stamp = ros::Time();
     marker_cam.header.frame_id = "world";
     marker_cam.ns = "camera";
     marker_cam.id = 1;
     marker_cam.type = visualization_msgs::Marker::ARROW;
     marker_cam.action = visualization_msgs::Marker::ADD;
    // cout<<x_cam<<" "<<y_cam<<" "<<z_cam<<endl;
     marker_cam.pose.position.x = x_cam;
     marker_cam.pose.position.y = y_cam;
     marker_cam.pose.position.z = z_cam+init_height;
     marker_cam.pose.orientation.x = q[0];
     marker_cam.pose.orientation.y = q[1];
     marker_cam.pose.orientation.z = q[2];
     marker_cam.pose.orientation.w = q[3];
     marker_cam.scale.x = 0.1;
     marker_cam.scale.y = 0.025;
     marker_cam.scale.z = 0.025;
     marker_cam.color.r = 1.0f;
     marker_cam.color.g = 0.0f;
     marker_cam.color.b = 0.0f;
     marker_cam.color.a = 1.0f;
     marker_cam.lifetime = ros::Duration();
     teach_cam_markers.push_back(marker_cam);
 //    cout<<"size of cam poses are: "<<teach_cam_markers.size()<<endl; 
}



void new_save_poses(float x_cam, float y_cam, float z_cam, vector<float> q){

     visualization_msgs::Marker new_marker_cam;
     new_marker_cam.header.stamp = ros::Time();
     new_marker_cam.header.frame_id = "world";
     new_marker_cam.ns = "camera";
     new_marker_cam.id = 1;
     new_marker_cam.type = visualization_msgs::Marker::ARROW;
     new_marker_cam.action = visualization_msgs::Marker::ADD;
    // cout<<x_cam<<" "<<y_cam<<" "<<z_cam<<endl;
     new_marker_cam.pose.position.x = x_cam;
     new_marker_cam.pose.position.y = y_cam;
     new_marker_cam.pose.position.z = z_cam+init_height;
     new_marker_cam.pose.orientation.x = q[0];
     new_marker_cam.pose.orientation.y = q[1];
     new_marker_cam.pose.orientation.z = q[2];
     new_marker_cam.pose.orientation.w = q[3];
     new_marker_cam.scale.x = 0.2;
     new_marker_cam.scale.y = 0.05;
     new_marker_cam.scale.z = 0.05;
     new_marker_cam.color.r = 0.0f;
     new_marker_cam.color.g = 1.0f;
     new_marker_cam.color.b = 0.0f;
     new_marker_cam.color.a = 1.0f;
     new_marker_cam.lifetime = ros::Duration();
     new_all_cam_markers.push_back(new_marker_cam);
//     cout<<"size of cam poses are: "<<new_all_cam_markers.size()<<endl;
     if(new_all_cam_markers.size() > 1){
        new_all_cam_markers.erase(new_all_cam_markers.begin());
     }
 
}

void new_save_poses_nodrift(float x_cam, float y_cam, float z_cam, vector<float> q){

     visualization_msgs::Marker new_marker_cam;
     new_marker_cam.header.stamp = ros::Time();
     new_marker_cam.header.frame_id = "world";
     new_marker_cam.ns = "camera";
     new_marker_cam.id = 1;
     new_marker_cam.type = visualization_msgs::Marker::ARROW;
     new_marker_cam.action = visualization_msgs::Marker::ADD;
    // cout<<x_cam<<" "<<y_cam<<" "<<z_cam<<endl;
     new_marker_cam.pose.position.x = x_cam;
     new_marker_cam.pose.position.y = y_cam;
     new_marker_cam.pose.position.z = z_cam+init_height;
     new_marker_cam.pose.orientation.x = q[0];
     new_marker_cam.pose.orientation.y = q[1];
     new_marker_cam.pose.orientation.z = q[2];
     new_marker_cam.pose.orientation.w = q[3];
     new_marker_cam.scale.x = 0.2;
     new_marker_cam.scale.y = 0.05;
     new_marker_cam.scale.z = 0.05;
     new_marker_cam.color.r = 0.0f;
     new_marker_cam.color.g = 1.0f;
     new_marker_cam.color.b = 0.0f;
     new_marker_cam.color.a = 1.0f;
     new_marker_cam.lifetime = ros::Duration();
     new_all_cam_markers_nodrift.push_back(new_marker_cam);
//     cout<<"size of cam poses are: "<<new_all_cam_markers.size()<<endl;
     if(new_all_cam_markers_nodrift.size() > 1){
        new_all_cam_markers_nodrift.erase(new_all_cam_markers_nodrift.begin());
     }
 
}


cv::Mat TransformFromMat(cv::Mat position_mat) {
     cv::Mat rotation(3,3,CV_32F);
     cv::Mat translation(3,1,CV_32F);

     rotation = position_mat.rowRange(0,3).colRange(0,3);
     translation = position_mat.rowRange(0,3).col(3);

     float orb_to_ros[3][3] = {{0,0,1},
                               {-1,0,0},
                               {0,-1,0}};


     cv::Mat ORB_ROS = cv::Mat(3, 3, CV_32FC1, &orb_to_ros);

     //Transform from orb coordinate system to ros coordinate system on camera coordinates
     cv::Mat camera_rotation = ORB_ROS * ((ORB_ROS*rotation).t());
     cv::Mat camera_translation = -ORB_ROS * ((ORB_ROS*rotation).t())*(ORB_ROS*translation);
     cv::Mat Transform_matrix(3,4,CV_32F);
     cv::hconcat(camera_rotation, camera_translation, Transform_matrix);

//     cout<<position_mat<<endl;
//     cv::Mat Transform_matrix = ORB_ROS * position_mat;
     return Transform_matrix;
}

std::vector<float> toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}

Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

/*tf::Transform TransformFromMat (cv::Mat position_mat) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  rotation = position_mat.rowRange(0,3).colRange(0,3);
  translation = position_mat.rowRange(0,3).col(3);

  tf::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                    rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                    rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
                                   );

  tf::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf::Transform (tf_camera_rotation, tf_camera_translation);
}
*/
/*
void visualize_obj(float x_obj,float y_obj,float z_obj)
{
     visualization_msgs::Marker marker_obj;
     marker_obj.header.stamp = ros::Time();
     marker_obj.header.frame_id = "map";
     marker_obj.ns = "object";
     marker_obj.id = 0;
     marker_obj.type = visualization_msgs::Marker::CUBE;
     marker_obj.action = visualization_msgs::Marker::ADD;
     marker_obj.pose.position.x = x_obj;
     marker_obj.pose.position.y = y_obj;
     marker_obj.pose.position.z = z_obj;
     marker_obj.pose.orientation.x = 0;
     marker_obj.pose.orientation.y = 0;
     marker_obj.pose.orientation.z = 0;
     marker_obj.pose.orientation.w = 0;
     marker_obj.scale.x = 0.5;
     marker_obj.scale.y = 0.5;
     marker_obj.scale.z = 0.5;
     marker_obj.color.r = 0.0f;
     marker_obj.color.g = 1.0f;
     marker_obj.color.b = 0.0f;
     marker_obj.color.a = 1.0f;
     marker_obj.lifetime = ros::Duration();
     all_markers.push_back(marker_obj);
     marker_pub.publish(all_markers);
}



void visualize_cam(float x_cam, float y_cam, float z_cam, vector<float> q)
{

     visualization_msgs::Marker marker_cam;
     marker_cam.header.stamp = ros::Time();
     marker_cam.header.frame_id = "world";
     marker_cam.ns = "camera";
     marker_cam.id = 1;
     marker_cam.type = visualization_msgs::Marker::ARROW;
     marker_cam.action = visualization_msgs::Marker::ADD;
    // cout<<x_cam<<" "<<y_cam<<" "<<z_cam<<endl;
     marker_cam.pose.position.x = x_cam;
     marker_cam.pose.position.y = y_cam;
     marker_cam.pose.position.z = z_cam+init_height;
     marker_cam.pose.orientation.x = q[0];
     marker_cam.pose.orientation.y = q[1];
     marker_cam.pose.orientation.z = q[2];
     marker_cam.pose.orientation.w = q[3];
     marker_cam.scale.x = 0.2;
     marker_cam.scale.y = 0.05;
     marker_cam.scale.z = 0.05;
     marker_cam.color.r = 0.0f;
     marker_cam.color.g = 1.0f;
     marker_cam.color.b = 0.0f;
     marker_cam.color.a = 1.0f;
     marker_cam.lifetime = ros::Duration();
     marker_pub_cam.publish(marker_cam);
}
*/

void visualize_cam()
{
     Posesarr.markers.resize(all_cam_markers.size());
     for(size_t i=0;i<all_cam_markers.size();i++){
          if (i%30 ==0){
          visualization_msgs::Marker cur_cam = all_cam_markers[i];
          Posesarr.markers[i].header.stamp = ros::Time();
          Posesarr.markers[i].header.frame_id = "world";
          Posesarr.markers[i].ns = "camera";
          Posesarr.markers[i].id = i;
          Posesarr.markers[i].type = visualization_msgs::Marker::ARROW;
          Posesarr.markers[i].action = visualization_msgs::Marker::ADD;
          Posesarr.markers[i].pose.position.x = cur_cam.pose.position.x;
          Posesarr.markers[i].pose.position.y = cur_cam.pose.position.y;
          Posesarr.markers[i].pose.position.z = cur_cam.pose.position.z;
          Posesarr.markers[i].pose.orientation.x = cur_cam.pose.orientation.x;
          Posesarr.markers[i].pose.orientation.y = cur_cam.pose.orientation.y;
          Posesarr.markers[i].pose.orientation.z = cur_cam.pose.orientation.z;
          Posesarr.markers[i].pose.orientation.w = cur_cam.pose.orientation.w;
          Posesarr.markers[i].scale.x = cur_cam.scale.x;
          Posesarr.markers[i].scale.y = cur_cam.scale.y;
          Posesarr.markers[i].scale.z = cur_cam.scale.z;
          Posesarr.markers[i].color.r = 0.0f;
          Posesarr.markers[i].color.g = 1.0f;
          Posesarr.markers[i].color.b = 0.0f;
          Posesarr.markers[i].color.a = 1.0f;
          Posesarr.markers[i].lifetime = ros::Duration();
          }
     }
     marker_pub_cam_arr.publish(Posesarr);
}


void teach_visualize_cam()
{
     teach_Posesarr.markers.resize(teach_cam_markers.size());
     for(size_t i=0;i<teach_cam_markers.size();i++){

          visualization_msgs::Marker cur_cam = teach_cam_markers[i];
          teach_Posesarr.markers[i].header.stamp = ros::Time();
          teach_Posesarr.markers[i].header.frame_id = "world";
          teach_Posesarr.markers[i].ns = "camera";
          teach_Posesarr.markers[i].id = i;
          teach_Posesarr.markers[i].type = visualization_msgs::Marker::ARROW;
          teach_Posesarr.markers[i].action = visualization_msgs::Marker::ADD;
          teach_Posesarr.markers[i].pose.position.x = cur_cam.pose.position.x;
          teach_Posesarr.markers[i].pose.position.y = cur_cam.pose.position.y;
          teach_Posesarr.markers[i].pose.position.z = cur_cam.pose.position.z;
          teach_Posesarr.markers[i].pose.orientation.x = cur_cam.pose.orientation.x;
          teach_Posesarr.markers[i].pose.orientation.y = cur_cam.pose.orientation.y;
          teach_Posesarr.markers[i].pose.orientation.z = cur_cam.pose.orientation.z;
          teach_Posesarr.markers[i].pose.orientation.w = cur_cam.pose.orientation.w;
          teach_Posesarr.markers[i].scale.x = cur_cam.scale.x;
          teach_Posesarr.markers[i].scale.y = cur_cam.scale.y;
          teach_Posesarr.markers[i].scale.z = cur_cam.scale.z;
          teach_Posesarr.markers[i].color.r = 1.0f;
          teach_Posesarr.markers[i].color.g = 0.0f;
          teach_Posesarr.markers[i].color.b = 0.0f;
          teach_Posesarr.markers[i].color.a = 1.0f;
          teach_Posesarr.markers[i].lifetime = ros::Duration();
     }
     teach_marker_pub_cam_arr.publish(teach_Posesarr);
     teach_cam_markers.clear();
}



void new_visualize_cam()
{
     New_Posesarr.markers.resize(new_all_cam_markers.size());
     for(size_t i=0;i<new_all_cam_markers.size();i++){

          visualization_msgs::Marker cur_cam = new_all_cam_markers[i];
          New_Posesarr.markers[i].header.stamp = ros::Time();
          New_Posesarr.markers[i].header.frame_id = "world";
          New_Posesarr.markers[i].ns = "camera";
          New_Posesarr.markers[i].id = i;
          New_Posesarr.markers[i].type = visualization_msgs::Marker::ARROW;
          New_Posesarr.markers[i].action = visualization_msgs::Marker::ADD;
          New_Posesarr.markers[i].pose.position.x = cur_cam.pose.position.x;
          New_Posesarr.markers[i].pose.position.y = cur_cam.pose.position.y;
          New_Posesarr.markers[i].pose.position.z = cur_cam.pose.position.z;
          New_Posesarr.markers[i].pose.orientation.x = cur_cam.pose.orientation.x;
          New_Posesarr.markers[i].pose.orientation.y = cur_cam.pose.orientation.y;
          New_Posesarr.markers[i].pose.orientation.z = cur_cam.pose.orientation.z;
          New_Posesarr.markers[i].pose.orientation.w = cur_cam.pose.orientation.w;
          New_Posesarr.markers[i].scale.x = cur_cam.scale.x;
          New_Posesarr.markers[i].scale.y = cur_cam.scale.y;
          New_Posesarr.markers[i].scale.z = cur_cam.scale.z;
          New_Posesarr.markers[i].color.r = 0.0f;
          New_Posesarr.markers[i].color.g = 0.0f;
          New_Posesarr.markers[i].color.b = 1.0f;
          New_Posesarr.markers[i].color.a = 1.0f;
          New_Posesarr.markers[i].lifetime = ros::Duration();
     }
     new_marker_pub_cam_arr.publish(New_Posesarr);
}

void new_visualize_cam_nodrift()
{
     New_Posesarr_nd.markers.resize(new_all_cam_markers_nodrift.size());
     for(size_t i=0;i<new_all_cam_markers_nodrift.size();i++){

          visualization_msgs::Marker cur_cam = new_all_cam_markers_nodrift[i];
          New_Posesarr_nd.markers[i].header.stamp = ros::Time();
          New_Posesarr_nd.markers[i].header.frame_id = "world";
          New_Posesarr_nd.markers[i].ns = "camera";
          New_Posesarr_nd.markers[i].id = i;
          New_Posesarr_nd.markers[i].type = visualization_msgs::Marker::ARROW;
          New_Posesarr_nd.markers[i].action = visualization_msgs::Marker::ADD;
          New_Posesarr_nd.markers[i].pose.position.x = cur_cam.pose.position.x;
          New_Posesarr_nd.markers[i].pose.position.y = cur_cam.pose.position.y;
          New_Posesarr_nd.markers[i].pose.position.z = cur_cam.pose.position.z;
          New_Posesarr_nd.markers[i].pose.orientation.x = cur_cam.pose.orientation.x;
          New_Posesarr_nd.markers[i].pose.orientation.y = cur_cam.pose.orientation.y;
          New_Posesarr_nd.markers[i].pose.orientation.z = cur_cam.pose.orientation.z;
          New_Posesarr_nd.markers[i].pose.orientation.w = cur_cam.pose.orientation.w;
          New_Posesarr_nd.markers[i].scale.x = cur_cam.scale.x/1.2;
          New_Posesarr_nd.markers[i].scale.y = cur_cam.scale.y/1.2;
          New_Posesarr_nd.markers[i].scale.z = cur_cam.scale.z/1.2;
          New_Posesarr_nd.markers[i].color.r = 1.0f;
          New_Posesarr_nd.markers[i].color.g = 0.0f;
          New_Posesarr_nd.markers[i].color.b = 0.0f;
          New_Posesarr_nd.markers[i].color.a = 1.0f;
          New_Posesarr_nd.markers[i].lifetime = ros::Duration();
     }
     new_marker_pub_cam_arr_nd.publish(New_Posesarr_nd);
}


};

int main(int argc, char** argv) {

     ros::init(argc, argv, "map_visualizer");
     map_show rs;
     ros::Rate r(30);
     while (ros::ok()){
            ros::spinOnce();
            r.sleep();
     }
     return 0;
}
