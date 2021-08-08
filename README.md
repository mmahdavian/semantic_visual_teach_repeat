# semantic_visual_teach_repeat
This is a robotic package for a semantic mapping based algorithm for visual teach and repeat

In order to use this package you need to have ORB-SLAM installed with some changes. So first:

1. clone and install https://github.com/mmahdavian/ORB_SLAM3
I have used ZED2 camera and you can file 
3. You need a YOLOv3 model to detect objects in the environment. We have provided a trained model file you can use. It has been trained on 24 most common objects in the environment. Those objects are: person,bench,backpack,umbrella,handbag,suitcase,bottle,cup,bowl,banana,apple,chair,couch,bed,tv,laptop,mouse,remote,keyboard,microwave,oven,toaster,sink,clock
...
...
If you want to use your own Yolo model you need to modify "semantic_mapping/map.py" code. In gotdata function there are three variables named big_objs,medium_objs and small_objs. They specify general size of each class of objects detected by Yolo. For example Sofa is a big objects and banana is a small one. So based on your model and class numbers change the code.

3. You need to use darknet_ros package to publish detected objects by Yolo model to ROS. For this purpose you can use following link. Clone it in your catkin_ws/src:
https://github.com/mmahdavian/darknet_ros

4. In your catkin_ws/src open a terminal and: git clone https://github.com/mmahdavian/semantic_visual_teach_repeat.git
5. cd ../
6. catkin_make
7. roslaunch darknet_ros yolo_v3.launch
8.  

