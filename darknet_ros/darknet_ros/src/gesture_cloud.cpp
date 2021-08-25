// #include <array>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <sensor_msgs/Image.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <visualization_msgs/Marker.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <math.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/console/time.h>
#include <pcl/common/angles.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/registration/icp.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/min_cut_segmentation.h>

#include <Eigen/Core>
#include <pcl/common/transforms.h>
//boost
#include <boost/make_shared.hpp>

//darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

// //server, client
#include "darknet_ros_msgs/pointing_vector.h"
#include "darknet_ros_msgs/AnchoringPoint.h"

//C++
#include <vector>
#include <iostream>
#include <algorithm>

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointTRGB;
typedef boost::shared_ptr<pcl::PointCloud<PointTRGB>> PointCloudTRGBPtr;
// typedef Matrix<float, 4, 4> Matrix4f;

using namespace std;

struct Center2D
{
    int x;
    int y;
};

struct Center3D
{
    float x;
    float y;
    float z;
};

struct Higher3D
{
    float x;
    float y;
    float z;
};

struct Box2D
{
    int xmin;
    int ymin;
    int xmax;
    int ymax;
};

struct Sauce
{
    std::string sauce_class;
    float probability;
    Box2D box_pixel;
    Center2D center_pixel;
    Center3D center_point;
    Higher3D higher_point;
    PointCloudTRGBPtr depth_cloud;
    PointCloudTRGBPtr rgb_cloud;
    int pick_ranking;
    int num_of_clusters;
};

int img_width;
int img_height;

bool data_received_yolo = false;
bool data_received_depth = false;
bool data_received_rgb = false;
bool cloud_saved_depth = false;
bool cloud_saved_rgb = false;

bool input_cloud_from_file = false;

//all information in one scene
std::vector<Sauce> sauces_pre{};
std::vector<Sauce> sauces_pre1{};
std::vector<Sauce> sauces_all{};
int check_num = 0;
int loop_count = 0;
int s_key = 0;

pcl::PointCloud<PointTRGB>::Ptr depth_cloud_ori(new pcl::PointCloud<PointTRGB>);
pcl::PointCloud<PointTRGB>::Ptr rgb_cloud_ori(new pcl::PointCloud<PointTRGB>);
pcl::PointCloud<PointTRGB>::Ptr cloud_rgb_aligned2depth(new pcl::PointCloud<PointTRGB>);

//sauce clusters
std::vector<std::string> text_list{};

bool enable_pcl_visualizer = false; //[true] pcl_visualizer; [false] rviz
// boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer ("Top 3 Sauces"));

ros::Publisher top3_cloud_pub, pub_arrowMarker_ave, anchoring_point_pub;
sensor_msgs::PointCloud2 top3_clouds_msg;
darknet_ros_msgs::AnchoringPoint anchoring_point_msg;

//Higheset Sauce Infomation
std::string highest_sauce_class;
float highest_sauce_3D_point_x;
float highest_sauce_3D_point_y;
float highest_sauce_3D_point_z;

bool compare_highest_points(const Sauce &sauce1, const Sauce &sauce2) //minimum z is the higheset!!!!
{
    return sauce1.center_point.z < sauce2.center_point.z;
}
bool compare_higher_points(PointTRGB com_point1, PointTRGB com_point2) //minimum z is the higheset!!!!
{
    return com_point1.x < com_point2.x;
}
// void camera_extrinsic_cb (const realsense2_camera::Extrinsics& extrinsicsMsg)
// {
//     //=================================================================//
//     // Transformation from realsense depth to color coordinate system
//     // Subscribe "/camera/extrinsic/depth_to_color" topic
//     //=================================================================//

//     cout<< "\ncamera_extrinsic_cb\n";

//     matrix_depth2color = Eigen::Matrix4f::Identity();
//     matrix_depth2color <<
//     extrinsicsMsg.rotation[0], extrinsicsMsg.rotation[1], extrinsicsMsg.rotation[2], extrinsicsMsg.translation[0],
//     extrinsicsMsg.rotation[3], extrinsicsMsg.rotation[4], extrinsicsMsg.rotation[5], extrinsicsMsg.translation[1],
//     extrinsicsMsg.rotation[6], extrinsicsMsg.rotation[7], extrinsicsMsg.rotation[8], extrinsicsMsg.translation[2],
//     0.0, 0.0, 0.0, 1.0;

//     cout << "frame id =" << extrinsicsMsg.header.frame_id << endl;
//     cout << "matrix_depth2color = \n" << matrix_depth2color << endl;
// }

// ros::Publisher pub, top3_cloud_pub;
// sensor_msgs::PointCloud2 top3_clouds_msg;
// pcl::visualization::CloudViewer viewer("Cloud Viewer");
void yolo_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &boxes_msg)
{
    //==================================================//
    // Subscribe "/darknet_ros/bounding_boxes" topic
    //==================================================//
    int obj_num = boxes_msg->bounding_boxes.size();
    if (loop_count % 4 == 0 && obj_num == 2)
    {
        sauces_pre.resize(obj_num);

        for (int k = 0; k < obj_num; ++k)
        {
            std::string sauce_class = boxes_msg->bounding_boxes[k].Class;
            float probability = boxes_msg->bounding_boxes[k].probability;
            int xmin = boxes_msg->bounding_boxes[k].xmin;
            int xmax = boxes_msg->bounding_boxes[k].xmax;
            int ymin = boxes_msg->bounding_boxes[k].ymin;
            int ymax = boxes_msg->bounding_boxes[k].ymax;
            int center_x = int((xmin + xmax) / 2.0);
            int center_y = int((ymin + ymax) / 2.0);

            sauces_pre[k].sauce_class = sauce_class;
            sauces_pre[k].probability = probability;
            sauces_pre[k].box_pixel.xmin = xmin;
            sauces_pre[k].box_pixel.xmax = xmax;
            sauces_pre[k].box_pixel.ymin = ymin;
            sauces_pre[k].box_pixel.ymax = ymax;
            sauces_pre[k].center_pixel.x = center_x;
            sauces_pre[k].center_pixel.y = center_y;
        }

        int value1;
        int value2;
        float prob_val1 = 0.0;
        float prob_val2 = 0.0;
        bool point_key = false;
        bool face_key = false;
        for (int k = 0; k < obj_num; ++k)
        {
            if (sauces_pre[k].sauce_class == "point")
            {
                if (prob_val1 <= sauces_pre[k].probability)
                {
                    prob_val1 = sauces_pre[k].probability;
                    value1 = k;
                    point_key = true;
                }
            }
            else if (sauces_pre[k].sauce_class == "head")
            {
                if (prob_val2 <= sauces_pre[k].probability)
                {
                    prob_val2 = sauces_pre[k].probability;
                    value2 = k;
                    face_key = true;
                }
            }
        }
        if (loop_count == 0)
        {
            sauces_pre1.resize(2);
            sauces_pre1[0] = sauces_pre[value1];
            sauces_pre1[1] = sauces_pre[value2];
        }
        else if (loop_count != 0 && point_key == true && face_key == true)
        {
            float u = ((sauces_pre[0].center_pixel.y * sauces_pre[0].center_pixel.x - sauces_pre1[0].center_pixel.y * sauces_pre1[0].center_pixel.x) / abs(sauces_pre1[0].center_pixel.y * sauces_pre1[0].center_pixel.x));
            sauces_pre1[0] = sauces_pre[value1];
            sauces_pre1[1] = sauces_pre[value2];
            if (u <= 0.1)
            {
                ++check_num;
            }
            if (check_num == 2)
            {
                cout<<"sauces_all have content!\n";
                sauces_all.resize(2);
                sauces_all[0] = sauces_pre[value1];
                sauces_all[1] = sauces_pre[value2];
                check_num = 0;
                s_key = 1;
            }
        }
        else
        {
            cout<<"not taking head or point!!\n";
            // s_key = 0;
        }
    }
    else if(loop_count % 4 == 0)
    {
        cout<<"input shoud be 2 item\n";
        // s_key = 0;
    }
    ++loop_count;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::fromROSMsg(*input, *depth_cloud_ori);
    pcl::fromROSMsg(*input, *cloud_rgb_aligned2depth);
    img_width = depth_cloud_ori->width;
    img_height = depth_cloud_ori->height;

    pcl::io::savePCDFile<PointTRGB>("original.pcd", *depth_cloud_ori);
}

void get_point_part()
{
    //========================================//
    // Go over all EVERY Yolov4 detected sauces
    // save 2D, 3D sauce information
    //========================================//
    // if (!sauces_all.empty())
    if (s_key == 1)
    {
        cout << "try to get the contents!\n";
        for (int n = 1; n > -1; --n)
        {
            cout << "Sauce #" << n << ":" << sauces_all[n].sauce_class << endl;

            //=========================================//
            // Extract Sauce's Depth Cloud(Orgainized)
            // 2D pixel mapping to 3D points
            //=========================================//
            sauces_all[n].depth_cloud = boost::make_shared<pcl::PointCloud<PointTRGB>>();

            int xmin = sauces_all[n].box_pixel.xmin;
            int xmax = sauces_all[n].box_pixel.xmax;
            int ymin = sauces_all[n].box_pixel.ymin;
            int ymax = sauces_all[n].box_pixel.ymax;

            //Ensure the 2D pixels are inside image's max width, height
            // if(xmin < 0) xmin = 114;//186;//0;
            // if(ymin < 0) ymin = 40;//74;//0;
            // if(xmax > img_width-1) xmax = 723;//1085;//img_width-1;
            // if(ymax > img_height-1) ymax = 424;//648;//img_height-1;
            cout << "\timgwidth, imgHeight = " << img_width << ",  " << img_height << endl;
            cout << "\tPixel (xmin, xmax, ymin, ymax) = " << xmin << ", " << xmax << ", " << ymin << ", " << ymax << endl;

            //Map 2D pixel to 3D points
            for (int i = xmin; i < xmax; i++)
            {
                for (int j = ymin; j < ymax; j++)
                {
                    PointTRGB depth_pt = cloud_rgb_aligned2depth->at(i, j);
                    if (pcl_isfinite(depth_pt.x) && pcl_isfinite(depth_pt.y) && pcl_isfinite(depth_pt.z))
                    {
                        sauces_all[n].depth_cloud->push_back(depth_pt);
                    }
                }
            }
            cout << "\tExtract [depth_cloud] = " << sauces_all[n].depth_cloud->size() << endl;

            //==================================================//
            // Extract Sauce's RGB Cloud(Unorgainized)
            // Use Sauce's depth_cloud to find nearest neighbors
            // in rgb_cloud
            //==================================================//
            // Find neighbors within radius
            float search_radius = 0.005; //(i.e. 0.005 = 5mm)

            pcl::KdTreeFLANN<PointTRGB> kdtree;
            kdtree.setInputCloud(cloud_rgb_aligned2depth);

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            std::vector<int> IdxRadiusSearchAll_rgb_sauces;

            for (int nn = 0; nn < sauces_all[n].depth_cloud->size(); ++nn)
            {
                pointIdxRadiusSearch.clear();
                pointRadiusSquaredDistance.clear();

                PointTRGB search_pt = sauces_all[n].depth_cloud->points[nn];

                if (pcl_isfinite(search_pt.x) && pcl_isfinite(search_pt.y) && pcl_isfinite(search_pt.z))
                {
                    if (kdtree.radiusSearch(search_pt, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
                    {
                        IdxRadiusSearchAll_rgb_sauces.insert(IdxRadiusSearchAll_rgb_sauces.end(), pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
                    }
                }
            }

            // Remove duplicate indices
            //std::vector<int> last;
            sort(IdxRadiusSearchAll_rgb_sauces.begin(), IdxRadiusSearchAll_rgb_sauces.end(), greater<int>());
            IdxRadiusSearchAll_rgb_sauces.erase(std::unique(IdxRadiusSearchAll_rgb_sauces.begin(), IdxRadiusSearchAll_rgb_sauces.end()), IdxRadiusSearchAll_rgb_sauces.end());

            sauces_all[n].rgb_cloud = boost::make_shared<pcl::PointCloud<PointTRGB>>();
            for (int mm = 0; mm < IdxRadiusSearchAll_rgb_sauces.size(); mm++)
            {
                PointTRGB rgb_pt = cloud_rgb_aligned2depth->points[IdxRadiusSearchAll_rgb_sauces[mm]];
                sauces_all[n].rgb_cloud->push_back(rgb_pt);
            }
            pcl::PointCloud<PointTRGB>::Ptr outlier_filtered(new pcl::PointCloud<PointTRGB>);
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> Static; //创建滤波器对象
            Static.setInputCloud(sauces_all[n].depth_cloud);           //设置待滤波的点云
            Static.setMeanK(100);                                    //设置在进行统计时考虑查询点临近点数
            Static.setStddevMulThresh(1.0);                          //设置判断是否为离群点的阀值
            Static.filter(*outlier_filtered);
            sauces_all[n].rgb_cloud = outlier_filtered;
            cout << "\tExtract [rgb_cloud] = " << sauces_all[n].rgb_cloud->size() << endl;

            std::string fileName1 = "yolo" + boost::to_string(n) + ".pcd";
            pcl::io::savePCDFileASCII(fileName1, *outlier_filtered);

            //==========================================//
            // Get Center 3D Points
            // map 2D center_pixel to 3D center_point
            //==========================================//
            int center_x = sauces_all[n].center_pixel.x;
            int center_y = sauces_all[n].center_pixel.y;

            PointTRGB center_pt_3d = cloud_rgb_aligned2depth->at(center_x, center_y);
            cout << "\tCenter_pt_3d = " << center_pt_3d.x << ", " << center_pt_3d.y << ", " << center_pt_3d.z << endl;

            // if Center_pt_3d is NAN, use all cluster's points
            if (!pcl_isfinite(center_pt_3d.x) || !pcl_isfinite(center_pt_3d.y) || !pcl_isfinite(center_pt_3d.z))
            {
                // Eigen::Vector4f pcaCentroid;
                // pcl::compute3DCentroid(*outlier_filtered, pcaCentroid);
                // center_pt_3d.x = pcaCentroid(0);
                // center_pt_3d.y = pcaCentroid(1);
                // center_pt_3d.z = pcaCentroid(2);
                std::sort(sauces_all[n].depth_cloud->points.begin(), sauces_all[n].depth_cloud->points.end(), compare_higher_points);
                center_pt_3d.x = sauces_all[n].depth_cloud->points[0].x;
                center_pt_3d.y = sauces_all[n].depth_cloud->points[0].y;
                center_pt_3d.z = sauces_all[n].depth_cloud->points[0].z;

                cout << "\t**Center_pt_3d = " << center_pt_3d.x << ", " << center_pt_3d.y << ", " << center_pt_3d.z << endl;
            }
            std::sort(sauces_all[n].depth_cloud->points.begin(), sauces_all[n].depth_cloud->points.end(), compare_higher_points);
            center_pt_3d.x = sauces_all[n].depth_cloud->points[0].x;
            center_pt_3d.y = sauces_all[n].depth_cloud->points[0].y;
            center_pt_3d.z = sauces_all[n].depth_cloud->points[0].z;

            /*
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGB>);  
            pcl::PassThrough<pcl::PointXYZRGB> pass1;
            pass1.setInputCloud (sauces_all[n].depth_cloud);
            pass1.setFilterFieldName ("x");
            pass1.setFilterLimits (0.0, center_pt_3d.x);
            //pass.setFilterLimitsNegative (true);
            pass1.filter (*cluster);
            sauces_all[n].depth_cloud = cluster;
            */
/*
            // 法线
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            // 申明一个Min-cut的聚类对象
            pcl::MinCutSegmentation<pcl::PointXYZRGB> clustering;
            clustering.setInputCloud(sauces_all[n].depth_cloud); //设置输入
                //创建一个点云，列出所知道的所有属于对象的点
            // （前景点）在这里设置聚类对象的中心点（想想是不是可以可以使用鼠标直接选择聚类中心点的方法呢？）
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr foregroundPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointXYZRGB point;
            point.x = center_pt_3d.x;
            point.y = center_pt_3d.y;
            point.z = center_pt_3d.z;
            foregroundPoints->points.push_back(point);
            clustering.setForegroundPoints(foregroundPoints); //设置聚类对象的前景点

            //设置sigma，它影响计算平滑度的成本。它的设置取决于点云之间的间隔（分辨率）
            clustering.setSigma(0.25);//0.02
            // 设置聚类对象的半径.
            clustering.setRadius(0.1);//0.005

            //设置需要搜索的临近点的个数，增加这个也就是要增加边界处图的个数
            clustering.setNumberOfNeighbours(14);//10

            //设置前景点的权重（也就是排除在聚类对象中的点，它是点云之间线的权重，）
            clustering.setSourceWeight(0.8);//0.65

            std::vector<pcl::PointIndices> clusters;
            clustering.extract(clusters);

            std::cout << "Maximum flow is " << clustering.getMaxFlow() << "." << std::endl;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cont_nan_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

            int currentClusterNum = 1;
            for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
            {
                //设置聚类后点云的属性
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
                for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
                {
                    if (!isnan(sauces_all[n].depth_cloud->points[*point].rgb))
                    {
                        cluster->points.push_back(sauces_all[n].depth_cloud->points[*point]);
                    }
                }

                cluster->width = cluster->points.size();
                cluster->height = 1;
                cluster->is_dense = false;

                //保存聚类的结果
                if (cluster->points.size() <= 0)
                    break;
                if(currentClusterNum ==1){sauces_all[n].depth_cloud = cluster;}
                // sauces_all[n].depth_cloud = cluster;
                std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
                std::string fileName = "cluster" + boost::to_string(n) + ".pcd";
                pcl::io::savePCDFileASCII(fileName, *cluster);
                currentClusterNum++;
            }
*/
            // if Center_pt_3d is NAN, use all cluster's points
            // if (!pcl_isfinite(center_pt_3d.x) || !pcl_isfinite(center_pt_3d.y) || !pcl_isfinite(center_pt_3d.z))
            // {
            int total_points = sauces_all[n].depth_cloud->size();
            center_pt_3d.x = 0;
            center_pt_3d.y = 0;
            center_pt_3d.z = 0;

            for (int kk = 0; kk < total_points; ++kk)
            {
                PointTRGB pt = sauces_all[n].depth_cloud->points[kk];
                center_pt_3d.x += pt.x;
                center_pt_3d.y += pt.y;
                center_pt_3d.z += pt.z;
            }

            center_pt_3d.x /= total_points;
            center_pt_3d.y /= total_points;
            center_pt_3d.z /= total_points;
            if(n == 1){sauces_all[1].center_point.x = center_pt_3d.x;}

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGB>);  
            pcl::PassThrough<pcl::PointXYZRGB> pass1;
            pass1.setInputCloud (sauces_all[n].depth_cloud);
            pass1.setFilterFieldName ("x");
            pass1.setFilterLimits (-10.0, sauces_all[1].center_point.x);
            //pass.setFilterLimitsNegative (true);
            pass1.filter (*cluster);
            sauces_all[n].depth_cloud = cluster;

            total_points = sauces_all[n].depth_cloud->size();
            center_pt_3d.x = 0;
            center_pt_3d.y = 0;
            center_pt_3d.z = 0;

            for (int kk = 0; kk < total_points; ++kk)
            {
                PointTRGB pt = sauces_all[n].depth_cloud->points[kk];
                center_pt_3d.x += pt.x;
                center_pt_3d.y += pt.y;
                center_pt_3d.z += pt.z;
            }

            center_pt_3d.x /= total_points;
            center_pt_3d.y /= total_points;
            center_pt_3d.z /= total_points;

            cout << "\t**Center_pt_3d = " << center_pt_3d.x << ", " << center_pt_3d.y << ", " << center_pt_3d.z << endl;
            // }
            sauces_all[n].center_point.x = center_pt_3d.x;
            sauces_all[n].center_point.y = center_pt_3d.y;
            sauces_all[n].center_point.z = center_pt_3d.z;

            //find the higher point
            // if (sauces_all[n].depth_cloud->size() != 0)
            // {
            //     std::sort(sauces_all[n].depth_cloud->points.begin(), sauces_all[n].depth_cloud->points.end(), compare_higher_points);
            //     PointTRGB pt2 = sauces_all[n].depth_cloud->points[0];
            //     sauces_all[n].higher_point.x = pt2.x;
            //     sauces_all[n].higher_point.y = pt2.y;
            //     sauces_all[n].higher_point.z = pt2.z;
            // }
            // else
            // {
            //     cout << "not sorted\n";
            //     sauces_all[n].higher_point.x = 0;
            //     sauces_all[n].higher_point.y = 0;
            //     sauces_all[n].higher_point.z = 0;
            // }
        }

        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGB>);  
        // pcl::PassThrough<pcl::PointXYZRGB> pass1;
        // pass1.setInputCloud (sauces_all[0].depth_cloud);
        // pass1.setFilterFieldName ("x");
        // pass1.setFilterLimits (-10.0, sauces_all[1].center_point.x);
        // //pass.setFilterLimitsNegative (true);
        // pass1.filter (*cluster);
        // sauces_all[0].depth_cloud = cluster;

        // double t = 1 + sqrt(1+0.0036*(pow(sauces_all[1].center_point.x,2)+pow(sauces_all[1].center_point.y,2)+pow(sauces_all[1].center_point.z,2)));
        double t = 0.0016/sauces_all[1].center_point.x;
        // sauces_all[1].higher_point.x = sauces_all[1].center_point.x;
        // sauces_all[1].higher_point.y = sauces_all[1].center_point.y;
        // sauces_all[1].higher_point.z = sauces_all[1].center_point.z;
//consider 3D center
        sauces_all[1].higher_point.x = sauces_all[1].center_point.x+sauces_all[1].center_point.x*t;
        sauces_all[1].higher_point.y = sauces_all[1].center_point.y+sauces_all[1].center_point.y*t;
        sauces_all[1].higher_point.z = sauces_all[1].center_point.z+sauces_all[1].center_point.z*t;

        // sauces_all[1].center_point.x = sauces_all[1].center_point.x+sauces_all[1].center_point.x*t;
        // sauces_all[1].center_point.y = sauces_all[1].center_point.y+sauces_all[1].center_point.y*t;
        // sauces_all[1].center_point.z = sauces_all[1].center_point.z+sauces_all[1].center_point.z*t;
        // sauces_all[1].higher_point.x = sauces_all[1].center_point.x*cos(15*M_PI/180)+sauces_all[1].center_point.z*sin(15*M_PI/180);
        // sauces_all[1].higher_point.y = sauces_all[1].center_point.y;
        // sauces_all[1].higher_point.z = sauces_all[1].center_point.x*(-sin(15*M_PI/180))+sauces_all[1].center_point.z*cos(15*M_PI/180);

        std::vector<double> dit_0to1;
        double x, y, z, r, w1;
        for(int i  = 0; i < sauces_all[0].depth_cloud->size(); ++i)
        {
            double x0 = sauces_all[1].higher_point.x - sauces_all[0].depth_cloud->points[i].x;
            double y0 = sauces_all[1].higher_point.y - sauces_all[0].depth_cloud->points[i].y;
            double z0 = sauces_all[1].higher_point.z - sauces_all[0].depth_cloud->points[i].z;
            double dist = sqrt(fabs(pow(x0, 2.0) + pow(y0, 2.0) + pow(z0, 2.0)));
            dit_0to1.push_back(dist); 
        }
        int max_iter = std::max_element(dit_0to1.begin(),dit_0to1.end()) - dit_0to1.begin();
        sauces_all[0].higher_point.x = sauces_all[0].depth_cloud->points[max_iter].x;
        sauces_all[0].higher_point.y = sauces_all[0].depth_cloud->points[max_iter].y;
        sauces_all[0].higher_point.z = sauces_all[0].depth_cloud->points[max_iter].z;

        //======================================================//
        // Select highest sauce
        // Highest sauce criteria:
        // (1) smallest z value (closest to the camera)
        //======================================================//
        float c_angle = 0*M_PI/180;
        Eigen::Matrix4f matrix_rotation;
        matrix_rotation <<
        cosf(c_angle), -sinf(c_angle), 0.0, 0.0,
        sinf(c_angle), cosf(c_angle), 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;
        // base_point = Eigen::Vector4f::Identity();
        for(int i  = 0; i < 2; ++i)
        {
            Eigen::Vector4f base_point;
            base_point <<
            sauces_all[i].higher_point.x,
            sauces_all[i].higher_point.y,
            sauces_all[i].higher_point.z,
            1.0;
            Eigen::Vector4f cross_product = matrix_rotation*base_point;
            sauces_all[i].higher_point.x = cross_product(0);
            sauces_all[i].higher_point.y = cross_product(1);
            sauces_all[i].higher_point.z = cross_product(2);
        }
        std::vector<double> head{sauces_all[1].higher_point.x, sauces_all[1].higher_point.y, sauces_all[1].higher_point.z};
        std::vector<double> hand{sauces_all[0].higher_point.x, sauces_all[0].higher_point.y, sauces_all[0].higher_point.z};

        anchoring_point_msg.head_minimum_z = head;
        anchoring_point_msg.pointing_minimum_z = hand;
        anchoring_point_msg.is_done = true;

        anchoring_point_pub.publish(anchoring_point_msg);

        //========================================//
        // Visualize highest sauce on rgb_cloud
        //========================================//
        //Display Top 3 Sauces via Rviz
        pcl::PointCloud<PointTRGB>::Ptr top3_clouds(new pcl::PointCloud<PointTRGB>);

        for (int idx = 0; idx < sauces_all.size(); ++idx)
        {
            PointTRGB pt_target;
            pt_target.x = sauces_all[idx].center_point.x;
            pt_target.y = sauces_all[idx].center_point.y;
            pt_target.z = sauces_all[idx].center_point.z;

            *top3_clouds = *top3_clouds + *(sauces_all[idx].depth_cloud);

            //Plot Center Point on Top1 sauce
            if (idx == 0)
            {
                highest_sauce_class = sauces_all[idx].sauce_class;
                highest_sauce_3D_point_x = sauces_all[idx].center_point.x;
                highest_sauce_3D_point_y = sauces_all[idx].center_point.y;
                highest_sauce_3D_point_z = sauces_all[idx].center_point.z;

                cout << "\n\n"
                        << "*****Find Highest Sauce, Done!*****\n"
                        << "\thighest_sauce_class = " << highest_sauce_class << endl
                        << "\thighest_sauce_3D_point_x = " << highest_sauce_3D_point_x << endl
                        << "\thighest_sauce_3D_point_y = " << highest_sauce_3D_point_y << endl
                        << "\thighest_sauce_3D_point_z = " << highest_sauce_3D_point_z << "\n"
                        << "\tsauce_3D_point_x = " << sauces_all[0].higher_point.x << endl
                        << "\tsauce_3D_point_y = " << sauces_all[0].higher_point.y << endl
                        << "\tsauce_3D_point_z = " << sauces_all[0].higher_point.z << "\n\n\n";
            }
            pcl::io::savePCDFile<PointTRGB>("g3.pcd", *top3_clouds);

            //Publish pcl::PointCloud to ROS sensor::PointCloud2, and to topic
            pcl::toROSMsg(*top3_clouds, top3_clouds_msg);
            top3_clouds_msg.header.frame_id = "map";

            top3_cloud_pub.publish(top3_clouds_msg);
        }
    }
    else
    {
        cout<<"enpty!";
        anchoring_point_msg.is_done = false;
        anchoring_point_pub.publish(anchoring_point_msg);
    }
}

void get_arrow()
{
    //   if (sauces_all[0].higher_point.z!=0){
    if (s_key == 1)
    {
        visualization_msgs::Marker::Ptr arrowMarker_ave(new visualization_msgs::Marker);
        // arrowMarker_ave->header = input->header;
        arrowMarker_ave->type = visualization_msgs::Marker::ARROW;
        arrowMarker_ave->action = visualization_msgs::Marker::ADD;
        arrowMarker_ave->color.a = 1.0;
        arrowMarker_ave->color.r = 0.0;
        arrowMarker_ave->color.g = 1.0;
        arrowMarker_ave->color.b = 0.0;
        arrowMarker_ave->scale.x = 0.01;
        arrowMarker_ave->scale.y = 0.1;
        arrowMarker_ave->scale.z = 0.1;

        arrowMarker_ave->points.resize(2);
        arrowMarker_ave->points[0].x = sauces_all[1].higher_point.x;
        arrowMarker_ave->points[0].y = sauces_all[1].higher_point.y;
        arrowMarker_ave->points[0].z = sauces_all[1].higher_point.z;

        arrowMarker_ave->points[1].x = sauces_all[0].higher_point.x;
        arrowMarker_ave->points[1].y = sauces_all[0].higher_point.y;
        arrowMarker_ave->points[1].z = sauces_all[0].higher_point.z;

        arrowMarker_ave->colors.resize(2);
        arrowMarker_ave->colors[0].r = 255;
        arrowMarker_ave->colors[0].g = 0;
        arrowMarker_ave->colors[0].b = 0;
        arrowMarker_ave->colors[0].a = 1;

        arrowMarker_ave->colors[1].r = 255;
        arrowMarker_ave->colors[1].g = 0;
        arrowMarker_ave->colors[1].b = 0;
        arrowMarker_ave->colors[1].a = 1;

        pub_arrowMarker_ave.publish(arrowMarker_ave);
    }
}

// bool response_highest(darknet_ros_msgs::pointing_vector::Request &req, darknet_ros_msgs::pointing_vector::Response &res)
// {
//     res.is_done = false;

//     bool highest_get = false;

//     highest_get = get_point_part();

//     if (highest_get == true && req.done == true)
//     {
//         std::vector<double> head{sauces_all[1].higher_point.x, sauces_all[1].higher_point.y, sauces_all[1].higher_point.z};
//         std::vector<double> hand{sauces_all[0].higher_point.x, sauces_all[0].higher_point.y, sauces_all[0].higher_point.z};

//         cout << "highest_get == true" << endl;
//         res.head_minimum_z = head;
//         res.pointing_minimum_z = hand;
//         res.is_done = true;
//         cout << "[SERVER] sending back response (highest_sauce)" << res.pointing_minimum_z[0] << endl;
//         sauces_all = {};
//         cout << "end" << endl;
//         s_key = 0;
//     }
//     else
//     {
//         //NO SAUCE OBJECT; OR UNABLE TO DETECT SAUCES
//         cout << "!!!!!!!!!!!!!!NO SAUCE OBJECT; OR UNABLE TO DETECT SAUCES!!!!!!!!!!!!!!\n"
//              << endl;
//         cout << "!!!!!!!!!!!!!!NO SAUCE OBJECT; OR UNABLE TO DETECT SAUCES!!!!!!!!!!!!!!\n"
//              << endl;
//         cout << "!!!!!!!!!!!!!!NO SAUCE OBJECT; OR UNABLE TO DETECT SAUCES!!!!!!!!!!!!!!\n"
//              << endl;
//     }
//     return true;
// }

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub_pointcloud = nh.subscribe("/zed/zed_node/point_cloud/cloud_registered", 10, cloud_cb);
    // ros::Subscriber sub_getarrow = nh.subscribe("/zed/zed_node/point_cloud/cloud_registered", 1, get_arrow);
    ros::Subscriber sub_yolov4 = nh.subscribe("/darknet_ros/bounding_boxes", 10, yolo_cb);
    top3_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/top3_cloud_pub", 10);
    pub_arrowMarker_ave = nh.advertise<visualization_msgs::Marker>("/got_arrow_pub", 10);
    anchoring_point_pub = nh.advertise<darknet_ros_msgs::AnchoringPoint>("/anchoring_point_pub", 10);
    // ros::ServiceServer service = nh.advertiseService("add_two_ints", response_highest);

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        if(s_key == 1)
        {
            get_point_part();
            get_arrow();
            s_key = 0;
            sauces_all = {};
        }
        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
