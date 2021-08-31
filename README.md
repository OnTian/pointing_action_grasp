# pointing_action_pick-and-place
# overview
This is a ROS package developed for object grasping & pointing action recognation in RGB-D images from ZED Stereo Camera. In the following ROS package you are able to use YOLO (V4) with trained model, it is able to detect pre-trained classes of the face & hand. Based on thier position to get the pointing vector in 3d space, then compare the distance between the object center & the pointing vector, 決定讓機器人要夾取的目標物件
For more information about YOLO, Darknet, please check the following link: YOLO: Real-Time Object Detection.

# environment
ubuntu 16.04(ROS kinetic)
python 3.6.12
opencv 3.4.9(computer vision library)

# installation
git clone 此專案, 以及 https://github.com/stereolabs/zed-ros-wrapper.git 後
將 darknet_ros/darknet 直接替換成這個文件夹：git clone https://github.com/AlexeyAB/darknet
執行 catkin_make -DCMAKE_BUILD_TYPE=Release
可下載已訓練頭與手的權重(https://drive.google.com/file/d/1P-SKyLj08Fwy8Mq2Xy-1ZGMimz7EhHsz/view?usp=sharing)至 darknet_ros/darknet_ros/yolo_network_config/weights

# usage
執行 . devel/setup.bash
啟動 ZED Stereo Camera：roslaunch zed_wrapper zed.launch
啟動 YOLO：roslaunch darknet_ros yolov4_obj.launch
YOLO 會透過 ROS 來訂閱 ZED 的RGB-D images, 直接輸出頭與手的辨識結果
確認是否正常取得物件之點雲：rosrun darknet_ros check_cluster
執行 rosrun darknet_ros gesture_cloud 用來估計指向向量
執行 rosrun darknet_ros cluster_segmentation 用來物件分割及計算間距
執行 rosrun darknet_ros plotiing.py 用來顯示指向向量及物件中心
------將機器人作為 master 端------
執行 rosrun strategy pose_strategy. 可進行物件夾取
