
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
int i = 0;
int j = 0;
void R_callback(const sensor_msgs::Image::ConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8); //將ROS訊息中的圖象資訊提取，生成新cv型別的圖象，複製給CvImage指標
    }
    catch(cv_bridge::Exception& e)  //異常處理
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        // return;
    }
    cv::Mat img = cv_ptr->image;

	if (img.empty()) {
		cout<<"Couldn't load image...\n";
	}
    else{
        static const std::string INPUT = "Input";
        // cv::namedWindow(INPUT);
        // cv::imshow("OpenCV setuo demo", img);
        string dpath = "/home/onda/yolo_ws/src/cpimage/";
        string type = ".jpg";
        stringstream ss;
        ss<<dpath<<j<<type;
        string filename = ss.str();
        if (i%10==0){
            cv::imwrite(filename,img);
            j = j+1;
        }
        // cv::waitKey(0);
        i = i+1;
	}
}

int main(int argc, char** argv) {
    cout<<"Start\n";
    ros::init(argc, argv, "video");
    ros::NodeHandle n;
    ros::Subscriber Subcaputre = n.subscribe("/zed/zed_node/left/image_rect_color", 1, R_callback);

    // return 0;

    ros::spin();
}