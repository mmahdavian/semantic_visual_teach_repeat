# semantic_visual_teach_repeat
This is a robotic package for a semantic mapping based algorithm for visual teach and repeat

In order to use this package you need to have ORB-SLAM installed with some changes. So first:

1. Clone and install https://github.com/mmahdavian/ORB_SLAM3
I have used ZED2 camera and you can file 
3. You need a YOLOv3 model to detect objects in the environment. We have provided a trained model file you can use. It has been trained on 24 most common objects in the environment. Those objects are: person,bench,backpack,umbrella,handbag,suitcase,bottle,cup,bowl,banana,apple,chair,couch,bed,tv,laptop,mouse,remote,keyboard,microwave,oven,toaster,sink,clock
https://drive.google.com/file/d/1S49rcLiUPrg4bG95tJpmsuYaCCiGi_Of/view?usp=sharing

Download the file. You will move the file to darknet-ros package later. If you want to use your own Yolo model you need to modify "semantic_mapping/map.py" code. In gotdata function there are three variables named big_objs,medium_objs and small_objs. They specify general size of each class of objects detected by Yolo. For example Sofa is a big objects and banana is a small one. So based on your model and class numbers change the code.

3. You need to use darknet_ros package to publish detected objects by Yolo model to ROS. For this purpose you can use following link. Clone it in your catkin_ws/src:
https://github.com/mmahdavian/darknet_ros

Then move the downloaded model file to darknet_ros/darknet_ros/yolo_network_config/weights folder.

4. In your catkin_ws/src open a terminal and: git clone https://github.com/mmahdavian/semantic_visual_teach_repeat.git
5. cd ../
6. catkin_make
7. roslaunch darknet_ros yolo_v3.launch
8. Then run ORB-SLAM using:
rosrun ORB_SLAM3 Mono Vocabulary/ORBvoc.txt "your camera yaml file"
9. roslaunch semantic_visual_teach_repeat create_map.launch

Then you need to move the camera which will be your teach path and the (Visual Teach and Repeat)VTR algorithm will generate a semantic map of the environment for you as teach map.
10. In order to repeat the same motion, first you need to change the name of the generated map and path file from:
projection.csv to C1.csv and obj_map.csv to M1.csv


