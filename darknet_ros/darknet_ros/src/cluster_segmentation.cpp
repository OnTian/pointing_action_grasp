#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/grabcut_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/passthrough.h>

#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <pcl/common/transforms.h>

//boost
#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>
#include <iostream>
#include <algorithm>
#include <string.h>

//survis
#include "darknet_ros_msgs/pointing_vector.h"
#include "darknet_ros_msgs/plot_position.h"
#include "darknet_ros_msgs/AnchoringPoint.h"


using namespace std;

static ros::Publisher PubOutput;
static int ExampleNumber;
ros::Publisher marker_pub;

typedef pcl::PointXYZ PointT;
typedef boost::shared_ptr<pcl::PointCloud<PointT> > PointCloudTPtr;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);  

struct Center3D
{
    float x;
    float y;
    float z;
};

struct Sauce
{
    Center3D center_point;
    PointCloudTPtr depth_cloud;
    double pick_ranking;
    double z_angle;
};

// std::vector<std::string> sauce_type{};
// int num_sauce_types = 0;
vector<Sauce> sauces_all{};
vector<double> head_minimum;
vector<double> pointing_minimum;
bool receive_key = false;
int dist_iter;
int u = 0;

bool compare_target_points(const Sauce &sauce1, const Sauce &sauce2) //minimum distance is target!!!!
{ 
    return sauce1.pick_ranking < sauce2.pick_ranking;
}

void tf_broadcast(const std::string frame_id){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "camera_depth_optical_frame";
    transformStamped.child_frame_id = frame_id;
    transformStamped.transform.translation.x = 2.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
}

void plane(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const std::string frame_id){
    // PassThrough Filtering
    // Ref: http://www.pointclouds.org/documentation/tutorials/passthrough.php#passthrough
    // Ref: http://wiki.ros.org/perception_pcl/Tutorials

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    // Use pcl::PointXYZRGB to visualize segmentation.
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_extracted;

    pcl::fromROSMsg(*cloud_msg, cloud);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object

    

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    
    seg.setInputCloud (cloud.makeShared());
    seg.setMaxIterations(100);
    seg.setProbability(0.95);
    seg.segment (*inliers, *coefficients);
    
    if (inliers->indices.size () == 0)
    {
        ROS_ERROR("Could not estimate a planar model for the given dataset.");
        exit(-1);
    }
    
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    // cloud = *cloud_ptr;
    // cloud_ptr = cloud.makeShared();

    pcl::ExtractIndices<pcl::PointXYZRGB> extract; 
    extract.setInputCloud(cloud.makeShared());
    extract.setIndices(inliers);                 
    extract.setNegative(true);  //false: 篩選Index對應的點，true：過濾獲取Index之外的點                
    extract.filter(cloud_extracted);
/*
    cloud = cloud_extracted;
    pcl::ModelCoefficients::Ptr coefficients1 (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers1 (new pcl::PointIndices);
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    
    seg.setInputCloud (cloud.makeShared());
    seg.setMaxIterations(100);
    seg.setProbability(0.95);
    seg.segment (*inliers1, *coefficients1);
    
    if (inliers1->indices.size () == 0)
    {
        ROS_ERROR("Could not estimate a planar model for the given dataset.");
        exit(-1);
    }
    
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    // cloud = *cloud_ptr;
    // cloud_ptr = cloud.makeShared();

    // pcl::ExtractIndices<pcl::PointXYZRGB> extract; 
    extract.setInputCloud(cloud.makeShared());
    extract.setIndices(inliers1);                 
    extract.setNegative(true);  //false: 篩選Index對應的點，true：過濾獲取Index之外的點                
    extract.filter(cloud_extracted);
    */


    // for (size_t i = 0; i < inliers->indices.size (); ++i){
        // cloud.points[inliers->indices[i]].r = 0;
        // cloud.points[inliers->indices[i]].g = 0;
        // cloud.points[inliers->indices[i]].b = 255;
    // }

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_extracted, output);
    output.header.frame_id = frame_id;
    // // Publish the data

    pcl::io::savePCDFile("VoxelGrid.pcd", output);

    PubOutput.publish(output);
}

double p_line_3(vector<double> p1, vector<double> p2)
{
	double x, y, z, r, w1;
    double x0 = p2[0] - p1[0];
    double y0 = p2[1] - p1[1];
    double z0 = p2[2] - p1[2];
    double l = sqrt(x0*x0 + y0*y0 + z0*z0);
    double ram = x0/l;
    double myu = y0/l;
    double nyu = z0/l;
    int min_iter;
    vector<double> min_dis;
    // vector<double> min_dis_copy;

    for(int i = 0; i < sauces_all.size(); ++i)
    {
        x = sauces_all[i].center_point.x - p1[0];
        y = sauces_all[i].center_point.y - p1[1];
        z = sauces_all[i].center_point.z - p1[2];
        w1 = x0 * x + y0 * y + z0 * z;
        // double t = w1/(pow(x0,2)+ pow(y0,2)+ pow(z0,2));
        // vector<double> w{x0*t - x, y0*t - y, z0*t - z};
        // // r = sqrt(fabs(x * x + y * y + z * z - w * w));
        // r = sqrt(fabs(pow(w[0], 2) + pow(w[1], 2) + pow(w[2], 2)));
        double t = x/x0;
        vector<double> w{y-t*y0, z-t*z0};
        r = sqrt(pow(w[0], 2) + pow(w[1], 2));
        min_dis.push_back(r);
        sauces_all[i].pick_ranking = r;
    }
    // return *std::min_element(min_dis,min_dis.size());
    min_iter = std::min_element(min_dis.begin(),min_dis.end()) - min_dis.begin();
    // min_dis_copy = min_dis;
    // min_dis_copy.erase(min_iter);
    // min_iter[1] = std::min_element(min_dis_copy.begin(),min_dis_copy.end()) - min_dis_copy.begin();
    char filename[]="record.txt";
    fstream fp;
    fp.open(filename, ios::app);//開啟檔案
    if(!fp){//如果開啟檔案失敗，fp為0；成功，fp為非0
        cout<<"Fail to open file: "<<endl;
    }
    x = sauces_all[min_iter].center_point.x - p1[0];
    y = sauces_all[min_iter].center_point.y - p1[1];
    z = sauces_all[min_iter].center_point.z - p1[2];
    double min_cos = (x0*x+y0*y+z0*z)/((sqrt(pow(x0, 2.0) + pow(y0, 2.0) + pow(z0, 2.0)))*(sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0))));
    double min_angle = acos(min_cos)*180/M_PI;
    fp<<p1[0]<<';'<<p1[1]<<';'<<p1[2]<<';'
        <<p2[0]<<';'<<p2[1]<<';'<<p2[2]<<';'
        <<sauces_all[min_iter].center_point.x<<';'<<sauces_all[min_iter].center_point.y<<';'<<sauces_all[min_iter].center_point.z<<';'
        <<min_dis[min_iter]<<';'<<min_angle<<';'<<endl;
 
    fp.close();//關閉檔案
	return min_iter;
}
/*
void euclideanClusterExtraction(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const std::string frame_id){
      // Read in the cloud data  
    pcl::PCDReader reader;  
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);  
    pcl::fromROSMsg(*cloud_msg, *cloud);
}
*/
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
      // Read in the cloud data  
    // pcl::PCDReader reader;  
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);  
    pcl::fromROSMsg(*cloud_msg, *cloud);
}

void vector_cb(const darknet_ros_msgs::AnchoringPoint& data)
{
    head_minimum = data.head_minimum_z;
    pointing_minimum = data.pointing_minimum_z;
    receive_key = data.is_done;
}

bool culaster_part()
{
    if(receive_key == true)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile("extract_object.pcd", *outlier_filtered);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile("cloud_filtered.pcd", *cloud_filtered);

        // Creating the KdTree object for the search method of the extraction  
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (outlier_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.005); // 2cm
        ec.setMinClusterSize (700);
        ec.setMaxClusterSize (8000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (outlier_filtered);
        ec.extract (cluster_indices); 

        int j = 0;  
        float colors[6][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};  
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);  
        pcl::copyPointCloud(*outlier_filtered, *cloud_cluster);  

        //due to save cluster
        int obj_num = cluster_indices.size();
        sauces_all.resize(obj_num);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)  
        {  
            //
            sauces_all[j].depth_cloud = boost::make_shared<pcl::PointCloud<PointT> >();

            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {  
                //
                PointT depth_pt = outlier_filtered->points[*pit];

                cloud_cluster->points[*pit].r = colors[j%6][0];  
                cloud_cluster->points[*pit].g = colors[j%6][1];  
                cloud_cluster->points[*pit].b = colors[j%6][2];  
                //
                sauces_all[j].depth_cloud->push_back(depth_pt);
            }  

            //
            int total_points = sauces_all[j].depth_cloud->size();
            PointT center_pt_3d;
            center_pt_3d.x = 0;
            center_pt_3d.y = 0;
            center_pt_3d.z = 0;

            for(int kk = 0; kk < total_points; ++kk)
            {
                PointT pt = sauces_all[j].depth_cloud->points[kk];
                center_pt_3d.x += pt.x;
                center_pt_3d.y += pt.y;
                center_pt_3d.z += pt.z;
            }

            center_pt_3d.x /= total_points;
            center_pt_3d.y /= total_points;
            center_pt_3d.z /= total_points;

            cout << "\t**Center_pt_3d = " << center_pt_3d.x << ", " << center_pt_3d.y << ", " << center_pt_3d.z << endl;

            sauces_all[j].center_point.x = center_pt_3d.x;
            sauces_all[j].center_point.y = center_pt_3d.y;
            sauces_all[j].center_point.z = center_pt_3d.z; 

            //
            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;  
            // std::stringstream ss;  
            // ss << "cloud_cluster_" << j << ".pcd";  
            // writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
            j++;  
        }  
    // viewer.showCloud (cloud_cluster);   
    // while (!viewer.wasStopped())  
    // {  
    //     sleep (1);  
    // }  

    /////// using servis //////////
    // bool servis_loop = true; 

    // while (servis_loop == true)
    // {
    //     darknet_ros_msgs::pointing_vector srv;
    //     srv.request.done = true;
    //     if (ros::service::call("add_two_ints",srv))
    //     {
    //         head_minimum = srv.response.head_minimum_z;
    //         pointing_minimum = srv.response.pointing_minimum_z;
    //         if(srv.response.is_done == true);
    //         {
    //             // cout<<"loop to close\n";
    //             servis_loop = false;
    //         }
    //     }
    //     else
    //     {
    //         // ROS_ERROR("Failed to call service add_two_ints");
    //     }
    // }

        dist_iter = p_line_3(head_minimum, pointing_minimum);
        std::sort(sauces_all.begin(), sauces_all.end(), compare_target_points);
        
        for (u = 0; u < sauces_all.size(); ++u)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            *cluster_cloud = *sauces_all[u].depth_cloud;

            //get more detail of grasping target
            pcl::VoxelGrid<pcl::PointXYZ> vg;  
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>);
            vg.setInputCloud (cluster_cloud);  
            vg.setLeafSize (0.01, 0.01, 0.01);  
            vg.filter (*cloud_filtered1); 
            pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_filtered1 (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> Static;   //创建滤波器对象
            Static.setInputCloud (cloud_filtered1);                              //设置待滤波的点云
            Static.setMeanK (100);                                         //设置在进行统计时考虑查询点临近点数
            Static.setStddevMulThresh (1.0);                              //设置判断是否为离群点的阀值
            Static.filter(*outlier_filtered1);
            *sauces_all[u].depth_cloud = *outlier_filtered1;
            pcl::io::savePCDFile("renew_cluster.pcd", *outlier_filtered1);

            // pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>());
        
            // std::string fileName(argv[1]);
            // pcl::io::loadPCDFile(fileName, *cloud);
        
            Eigen::Vector4f pcaCentroid;
            pcl::compute3DCentroid(*outlier_filtered1, pcaCentroid);
            Eigen::Matrix3f covariance;
            pcl::computeCovarianceMatrixNormalized(*outlier_filtered1, pcaCentroid, covariance);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
            Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
            Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
            eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
            eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
            eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

            // Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
            // transform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
            // transform.block<3, 1>(0, 3) = -1.0f * (transform.block<3,3>(0,0)) * (pcaCentroid.head<3>());// 
        
            // pcl::PointCloud<PointT>::Ptr transformedCloud(new pcl::PointCloud<PointT>);
            // pcl::transformPointCloud(*outlier_filtered1, *transformedCloud, transform);
        
            // std::cout << eigenValuesPCA << std::endl;
            // std::cout << eigenVectorsPCA << std::endl;
        
            //转换到原点时的主方向
            // PointT o;
            // o.x = 0.0;
            // o.y = 0.0;
            // o.z = 0.0;
            // Eigen::Affine3f tra_aff(transform);
            // Eigen::Vector3f pz = eigenVectorsPCA.col(0);
            // Eigen::Vector3f py = eigenVectorsPCA.col(1);
            // Eigen::Vector3f px = eigenVectorsPCA.col(2);
            // pcl::transformVector(pz, pz, tra_aff);
            // pcl::transformVector(py, py, tra_aff);
            // pcl::transformVector(px, px, tra_aff);
            // PointT pcaZ;
            // pcaZ.x = 1000 * pz(0);
            // pcaZ.y = 1000 * pz(1);
            // pcaZ.z = 1000 * pz(2);
            // PointT pcaY;
            // pcaY.x = 1000 * py(0);
            // pcaY.y = 1000 * py(1);
            // pcaY.z = 1000 * py(2);
            // PointT pcaX;
            // pcaX.x = 1000 * px(0);
            // pcaX.y = 1000 * px(1);
            // pcaX.z = 1000 * px(2);
        
            //初始位置时的主方向
            PointT c;
            c.x = pcaCentroid(0);
            c.y = pcaCentroid(1);
            c.z = pcaCentroid(2);
            PointT pcZ;
            pcZ.x = 0.2 * eigenVectorsPCA(0, 0) + c.x;
            pcZ.y = 0.2 * eigenVectorsPCA(1, 0) + c.y;
            pcZ.z = 0.2 * eigenVectorsPCA(2, 0) + c.z;
            PointT pcY;
            pcY.x = 0.2 * eigenVectorsPCA(0, 1) + c.x;
            pcY.y = 0.2 * eigenVectorsPCA(1, 1) + c.y;
            pcY.z = 0.2 * eigenVectorsPCA(2, 1) + c.z;
            PointT pcX;
            pcX.x = 0.2 * eigenVectorsPCA(0, 2) + c.x;
            pcX.y = 0.2 * eigenVectorsPCA(1, 2) + c.y;
            pcX.z = 0.2 * eigenVectorsPCA(2, 2) + c.z;
            // visualization
            // pcl::visualization::PCLVisualizer viewer;
            // // pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler(outlier_filtered1,255, 0, 0); 
            // // pcl::visualization::PointCloudColorHandlerCustom<PointT> tc_handler(transformedCloud, 0, 255, 0); 
            // viewer.addPointCloud(outlier_filtered1, "cloud");
            // // viewer.addPointCloud(transformedCloud,tc_handler,"transformCloud");
            
            // // viewer.addArrow(pcaZ, o, 0.0, 0.0, 1.0, false, "arrow_Z");
            // // viewer.addArrow(pcaY, o, 0.0, 1.0, 0.0, false, "arrow_Y");
            // // viewer.addArrow(pcaX, o, 1.0, 0.0, 0.0, false, "arrow_X");

            // viewer.addArrow(pcZ, c, 0.0, 0.0, 1.0, false, "arrow_z");
            // viewer.addArrow(pcY, c, 0.0, 1.0, 0.0, false, "arrow_y");
            // viewer.addArrow(pcX, c, 1.0, 0.0, 0.0, false, "arrow_x");
        
            // // viewer.addCoordinateSystem(100);
            // viewer.setBackgroundColor(0.0, 0.0, 0.0);
            // while (!viewer.wasStopped())
            // {
            //     viewer.spinOnce(50);
            // }

            // pcl::visualization::CloudViewer viewer("cluster viewer");
            // viewer.showCloud (outlier_filtered1);   
            // while (!viewer.wasStopped())  
            // {  
            //     sleep (1);  
            // } 
            sauces_all[u].z_angle = atan2(eigenVectorsPCA(1, 2), eigenVectorsPCA(2, 2));
            int target_key = 0;
            cout<<"if selected the target is right enter 'true', if not enter 'false': \n";
            // cin>>target_key;
            if (target_key == 0)
            {
                break;
            }
        }
        cout<<"how u now:"<<u<<endl;
        sauces_all[u].pick_ranking = 1;
        visualization_msgs::Marker points, line_strip;
        points.header.frame_id = line_strip.header.frame_id = "map";
        points.header.stamp = line_strip.header.stamp = ros::Time::now();
        points.ns = line_strip.ns = "points_and_lines";
        points.action = line_strip.action = visualization_msgs::Marker::ADD;
        // points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

        points.id = 0;
        line_strip.id = 1;

        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;

        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        // Create the vertices for the points and lines
        geometry_msgs::Point p;

        p.x = sauces_all[u].center_point.x;
        p.y = sauces_all[u].center_point.y;
        p.z = sauces_all[u].center_point.z;
        points.points.push_back(p);
        line_strip.points.push_back(p);

        p.x = pointing_minimum[0];
        p.y = pointing_minimum[1];
        p.z = pointing_minimum[2];
        line_strip.points.push_back(p);
        
        marker_pub.publish(points);
        marker_pub.publish(line_strip);
        return true;
    }
    else
    {
        return false;
    }
    // sauces_all = {};

    /*
    // Euclidean Cluster Extraction
    // Ref: http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction
    
    enum COLOR_RGB{
        RED=0,
        GREEN,
        BLUE,
        COLOR_MAX
    };
    const int  GrabCutHelper(cloud_msg);
// the sensor_msgs/PointCloud2 data to pcl/PointCloud
    // Use pcl::PointXYZRGB to visualize segmentation.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>()); 
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>());
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01, 0.01, 0.01);
    vg.filter (*cloud_filtered);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_extracted(new pcl::PointCloud<pcl::PointXYZRGB>()); 
    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            ROS_WARN("Could not estimate a planar model for the given dataset.");
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_extracted);
        *cloud_filtered = *cloud_extracted;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>());
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int cluster_i=0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZRGB>());
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); 
            it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>());
        for (std::vector<int>::const_iterator pit = it->indices.begin (); 
                pit != it->indices.end (); ++pit){
            cloud_filtered->points[*pit].r = CLUSTER_COLOR[cluster_i][RED];
            cloud_filtered->points[*pit].g = CLUSTER_COLOR[cluster_i][GREEN];
            cloud_filtered->points[*pit].b = CLUSTER_COLOR[cluster_i][BLUE];
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        
        *cloud_output += *cloud_cluster;

        cluster_i++;
        if(cluster_i >= CLUSTER_MAX){
            break;
        }
    }

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_output, output);
    // pcl_conversions::moveFromPCL(cloud_extracted, output);
    output.header.frame_id = frame_id;
    pcl::io::savePCDFile("cluster1.pcd", output);
    printf("done\n");

    // Publish the data
    PubOutput.publish(output);
    */
}

bool response_position(darknet_ros_msgs::plot_position::Request& req, darknet_ros_msgs::plot_position::Response& res)
{
    cout<<"start servis"<<endl;
    res.is_done = false;
    vector<double> center;
    bool eucli = culaster_part();

    if(eucli == true && sauces_all[u].pick_ranking == 1 && req.done == true)
    {
        cout<<"request is true"<<endl;
        for(int idx = 0; idx < sauces_all.size(); ++idx)
        { 
            center.push_back(sauces_all[idx].center_point.x);
            center.push_back(sauces_all[idx].center_point.y);
            center.push_back(sauces_all[idx].center_point.z);
        }
        res.head_minimum_z = head_minimum;
        res.pointing_minimum_z = pointing_minimum;
        //
        vector<double> target_pos{sauces_all[u].center_point.x, sauces_all[u].center_point.y, sauces_all[u].center_point.z};
        res.target = target_pos;
        res.object = center;
        res.angle = sauces_all[u].z_angle;
        res.is_done = true;

        sauces_all = {};
    }
    else
    {
        //NO SAUCE OBJECT; OR UNABLE TO DETECT SAUCES
        cout << "!!!!!!!!!!!!!!NO SAUCE OBJECT; OR UNABLE TO DETECT SAUCES!!!!!!!!!!!!!!\n" << endl;
        cout << "!!!!!!!!!!!!!!NO SAUCE OBJECT; OR UNABLE TO DETECT SAUCES!!!!!!!!!!!!!!\n" << endl;
        cout << "!!!!!!!!!!!!!!NO SAUCE OBJECT; OR UNABLE TO DETECT SAUCES!!!!!!!!!!!!!!\n" << endl;
    }  
    cout<<"end servis"<<endl;
    return true;
}

/*
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // printf("test2\n");
    const static std::string EXAMPLE_FRAME_ID = "example_frame";
   
    switch(ExampleNumber){
    case 0:
        plane(cloud_msg, EXAMPLE_FRAME_ID);
        break;
    case 1:
        break;
    case 2:  
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);  
        pcl::fromROSMsg(*cloud_msg, *cloud);
        // euclideanClusterExtraction(cloud_msg, EXAMPLE_FRAME_ID);
        break;
    default:
        break;
    }

    // to shift positions of rendering point clouds
    tf_broadcast(EXAMPLE_FRAME_ID);
}
*/

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "example_segmentation");
    ros::NodeHandle nh;
    // ros::NodeHandle nh("~");

    // nh.param<int>("number", ExampleNumber, 2);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/zed/zed_node/point_cloud/cloud_registered", 10, cloud_cb);
    ros::Subscriber vector_sub = nh.subscribe("/anchoring_point_pub", 10, vector_cb);

    ros::ServiceServer service = nh.advertiseService("show_figure", response_position);
    // ros::Subscriber pointer_sub = nh.subscribe("/show_figure", 10, response_position);
    // ros::Publisher point_vector_pub = nh.subscribe("/show_figure", 10, response_position);

    printf("test111\n");
    // Create a ROS publisher for the output point cloud
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        if(receive_key == true)
        {
            // cout<<"able to start\n";
            // ros::ServiceClient client = nh.serviceClient<darknet_ros_msgs::pointing_vector>("add_two_ints");
            marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
            PubOutput = nh.advertise<sensor_msgs::PointCloud2> ("/output", 10);
        }
        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

    // ros::spin ();
}
