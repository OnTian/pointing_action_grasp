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
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/passthrough.h>

#include <visualization_msgs/Marker.h>
//boost
#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>
#include <iostream>
#include <algorithm>

using namespace std;

// static ros::Publisher PubOutput;
// static int ExampleNumber;
// ros::Publisher marker_pub;

typedef pcl::PointXYZRGB PointT;
typedef boost::shared_ptr<pcl::PointCloud<PointT> > PointCloudTPtr;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);  
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);  

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
    int pick_ranking;
    int num_of_clusters;
};

// std::vector<std::string> sauce_type{};
// int num_sauce_types = 0;
vector<Sauce> sauces_all{};
vector<double> head_minimum;
vector<double> pointing_minimum;
int dist_iter;

// pcl::visualization::CloudViewer viewer("cluster viewer");  

pcl::SACSegmentation<pcl::PointXYZRGB> palne_to_E_cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // Create the segmentation object for the planar model and set all the parameters  
    // pcl::SACSegmentation<pcl::PointXYZRGB> seg;  
    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());  
    // pcl::PCDWriter writer;  
    // seg.setOptimizeCoefficients (true);  
    // seg.setModelType (pcl::SACMODEL_PLANE);  
    // seg.setMethodType (pcl::SAC_RANSAC);  
    // seg.setMaxIterations (150);  //100
    // seg.setDistanceThreshold (0.02);  // 0.02

    // seg.setInputCloud(cloud_filtered);  
    // seg.segment (*inliers, *coefficients); //*
    // cout<<"the plane is "<<coefficients->values.size() <<endl;  

    // if (inliers->indices.size () == 0){  
    //     ROS_ERROR("Could not estimate a planar model for the given dataset.");
    //     exit(-1);
    // }  

    // // Extract the planar inliers from the input cloud  
    // pcl::ExtractIndices<pcl::PointXYZRGB> extract;  
    // extract.setInputCloud (cloud_filtered);  
    // extract.setIndices (inliers);  
    // // extract.setNegative (false);  

    // // Remove the planar inliers, extract the rest  
    // extract.setNegative (true);  
    // extract.filter (*cloud_filtered); //*  

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr outlier_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> Static;   //创建滤波器对象
    // Static.setInputCloud (cloud_filtered);                              //设置待滤波的点云
    // Static.setMeanK (100);                                         //设置在进行统计时考虑查询点临近点数
    // Static.setStddevMulThresh (1.0);                              //设置判断是否为离群点的阀值
    // Static.filter(*outlier_filtered);

    // pcl::io::savePCDFile("extract_object.pcd", *outlier_filtered);

    // // Creating the KdTree object for the search method of the extraction  
    // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    // tree->setInputCloud (outlier_filtered);

    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    // ec.setClusterTolerance (0.01); // 2cm
    // ec.setMinClusterSize (3000);
    // ec.setMaxClusterSize (8000);
    // ec.setSearchMethod (tree);
    // ec.setInputCloud (outlier_filtered);
    // ec.extract (cluster_indices); 

    // int j = 0;  
    // float colors[6][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};  
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);  
    // pcl::copyPointCloud(*outlier_filtered, *cloud_cluster);  

    // //due to save cluster
    // int obj_num = cluster_indices.size();
    // sauces_all.resize(obj_num);

    // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)  
    // {  
    //     //
    //     sauces_all[j].depth_cloud = boost::make_shared<pcl::PointCloud<PointT> >();

    //     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {  
    //         //
    //         PointT depth_pt = cloud_filtered->points[*pit];

    //         cloud_cluster->points[*pit].r = colors[j%6][0];  
    //         cloud_cluster->points[*pit].g = colors[j%6][1];  
    //         cloud_cluster->points[*pit].b = colors[j%6][2];  
    //         //
    //         sauces_all[j].depth_cloud->push_back(depth_pt);
    //     }  
    //     //
    //     int total_points = sauces_all[j].depth_cloud->size();
    //     PointT center_pt_3d;
    //     center_pt_3d.x = 0;
    //     center_pt_3d.y = 0;
    //     center_pt_3d.z = 0;

    //     for(int kk = 0; kk < total_points; ++kk)
    //     {
    //         PointT pt = sauces_all[j].depth_cloud->points[kk];
    //         center_pt_3d.x += pt.x;
    //         center_pt_3d.y += pt.y;
    //         center_pt_3d.z += pt.z;
    //     }

    //     center_pt_3d.x /= total_points;
    //     center_pt_3d.y /= total_points;
    //     center_pt_3d.z /= total_points;

    //     cout << "\t**Center_pt_3d = " << center_pt_3d.x << ", " << center_pt_3d.y << ", " << center_pt_3d.z << endl;

    //     sauces_all[j].center_point.x = center_pt_3d.x;
    //     sauces_all[j].center_point.y = center_pt_3d.y;
    //     sauces_all[j].center_point.z = center_pt_3d.z; 

    //     //
    //     std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;  
    //     std::stringstream ss;  
    //     // ss << "cloud_cluster_" << j << ".pcd";  
    //     // writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
    //     j++;  
    // } 

    // // pcl::visualization::CloudViewer viewer ("Cluster viewer1");
    // viewer.showCloud (cloud_cluster);
    // while (!viewer.wasStopped ())
    // {
    //     sleep (1);
    // }
}

pcl::RegionGrowingRGB<pcl::PointXYZRGB> color_based_seg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);  

    // pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    // // pcl::IndicesPtr indices (new std::vector <int>);
    // pcl::PassThrough<pcl::PointXYZRGB> pass;
    // pass.setInputCloud (cloud);
    // pass.setFilterFieldName ("z");
    // pass.setFilterLimits (-3.0, 0.0);
    // pass.filter (*cloud_filtered);


    // pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
/*
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outlier_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> Static;   //创建滤波器对象
    Static.setInputCloud (cloud_filtered);                              //设置待滤波的点云
    Static.setMeanK (100);                                         //设置在进行统计时考虑查询点临近点数
    Static.setStddevMulThresh (1.5);                              //设置判断是否为离群点的阀值
    Static.filter(*outlier_filtered);

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;  
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());  
    pcl::PCDWriter writer;  
    seg.setOptimizeCoefficients (true);  
    seg.setModelType (pcl::SACMODEL_PLANE);  
    seg.setMethodType (pcl::SAC_RANSAC);  
    seg.setMaxIterations (100);  
    seg.setDistanceThreshold (0.02);  

    seg.setInputCloud(outlier_filtered);  
    // seg.setIndices (indices);
    seg.segment (*inliers, *coefficients); //*

    if (inliers->indices.size () == 0){  
        ROS_ERROR("Could not estimate a planar model for the given dataset.");
        exit(-1);
    }  

    // Extract the planar inliers from the input cloud  
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;  
    extract.setInputCloud (outlier_filtered);  
    extract.setIndices (inliers);  
    // extract.setNegative (false);  

    // Remove the planar inliers, extract the rest  
    extract.setNegative (true);  
    extract.filter (*cloud_plane); //* 

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outlier_filtered1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> Static1;   //创建滤波器对象
    Static1.setInputCloud (cloud_plane);                              //设置待滤波的点云
    Static1.setMeanK (100);                                         //设置在进行统计时考虑查询点临近点数
    Static1.setStddevMulThresh (0.7);                              //设置判断是否为离群点的阀值
    Static1.filter(*outlier_filtered1);*/
    // pcl::visualization::CloudViewer viewer ("Cluster viewer");
    // viewer.showCloud (outlier_filtered1);
/*
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (outlier_filtered1);
    // reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (10);
    reg.setPointColorThreshold (6);
    reg.setRegionColorThreshold (5);
    reg.setMinClusterSize (600);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

    //// calculating the mean normal for each cluster
    //std::vector<Eigen::Vector3f> mean_normals(clusters.size(), Eigen::Vector3f({ 0.,0.,0. }));
    //for (auto cluster_idx = 0; cluster_idx < clusters.size(); cluster_idx++)
    //{
    //    for (auto point_idx: clusters[cluster_idx].indices)
    //    {
    //       mean_normals[cluster_idx] += normals->points[point_idx].getNormalVector3fMap();
    //    }
    //    mean_normals[cluster_idx].normalize();
    //}
    // pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud (colored_cloud);


    while (!viewer.wasStopped ())
    {
        sleep (1);
    }
*/
}

pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> nomal_based_seg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
/*
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::Normal>::Ptr normals1 (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud_filtered2);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals1);

    //   pcl::IndicesPtr indices (new std::vector <int>);
    //   pcl::PassThrough<pcl::PointXYZ> pass;
    //   pass.setInputCloud (cloud_filtered);
    //   pass.setFilterFieldName ("z");
    //   pass.setFilterLimits (0.0, 1.0);
    //   pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize (50);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud_filtered2);
    //reg.setIndices (indices);
    reg.setInputNormals (normals1);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    std::vector <pcl::PointIndices> clusters1;
    reg.extract (clusters1);

    std::cout << "Number of clusters is equal to " << clusters1.size () << std::endl;
    std::cout << "First cluster has " << clusters1[0].indices.size () << " points." << std::endl;
    std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;
    int counter = 0;
    while (counter < clusters1[0].indices.size ())
    {
    std::cout << clusters1[0].indices[counter] << ", ";
    counter++;
    if (counter % 10 == 0)
        std::cout << std::endl;
    }
    std::cout << std::endl;

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
      */
    //   pcl::visualization::CloudViewer viewer ("Cluster viewer");
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);  
    // pcl::fromROSMsg(*cloud_msg, *cloud1);  

      // Read in the cloud data 
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm  
    // pcl::VoxelGrid<pcl::PointXYZ> vg;  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);  
    // vg.setInputCloud (cloud);  
    // vg.setLeafSize (0.01, 0.01, 0.01);  
    // vg.filter (*cloud_filtered);  
    // std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*  
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZRGB>);  
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    // pass.setInputCloud (cloud);
    // pass.setFilterFieldName ("x");
    // pass.setFilterLimits (0.0, 3.0);
    // //pass.setFilterLimitsNegative (true);
    // pass.filter (*cloud_filtered1);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB>);  
    // pass.setInputCloud (cloud_filtered1);
    // pass.setFilterFieldName ("y");
    // pass.setFilterLimits (-2.0, 2.0);
    // //pass.setFilterLimitsNegative (true);
    // pass.filter (*cloud_filtered2);
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-20.0, 0.0);
    pass.setNegative (false);
    pass.filter (*cloud_filtered);

    pcl::io::savePCDFile("cloud_filtered.pcd", *cloud_filtered);

    pcl::visualization::CloudViewer viewer("cluster viewer");
    // viewer.showCloud (cloud_filtered);
    // while (!viewer.wasStopped())
    // {
    //     sleep (1);
    // }

    ////////////////////////////////////////
    //  some segmentation method          //
    ////////////////////////////////////////
    // palne_to_E_cluster(cloud);
    // color_based_seg(cloud);
    // nomal_based_seg(cloud);

    // 法线
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // 申明一个Min-cut的聚类对象
    pcl::MinCutSegmentation<pcl::PointXYZRGB> clustering;
    clustering.setInputCloud(cloud_filtered);   //设置输入
        //创建一个点云，列出所知道的所有属于对象的点 
    // （前景点）在这里设置聚类对象的中心点（想想是不是可以可以使用鼠标直接选择聚类中心点的方法呢？）
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr foregroundPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointXYZRGB point;
    point.x = 100.0;
    point.y = 100.0;
    point.z = 100.0;
    foregroundPoints->points.push_back(point);
    clustering.setForegroundPoints(foregroundPoints);  //设置聚类对象的前景点
       
        //设置sigma，它影响计算平滑度的成本。它的设置取决于点云之间的间隔（分辨率）
    clustering.setSigma(0.02);
    // 设置聚类对象的半径.
    clustering.setRadius(0.01);

         //设置需要搜索的临近点的个数，增加这个也就是要增加边界处图的个数
    clustering.setNumberOfNeighbours(20);

        //设置前景点的权重（也就是排除在聚类对象中的点，它是点云之间线的权重，）
    clustering.setSourceWeight(0.6);

    std::vector <pcl::PointIndices> clusters;
    clustering.extract(clusters);

    std::cout << "Maximum flow is " << clustering.getMaxFlow() << "." << std::endl;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cont_nan_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

    int currentClusterNum = 1;
    for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
    {
        //设置聚类后点云的属性
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr nan_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
            {cont_nan_cluster->points.push_back(cloud_filtered->points[*point]);
            if (!isnan(cloud_filtered->points[*point].rgb))
                cluster->points.push_back(cloud_filtered->points[*point]);
            else
                nan_cluster->points.push_back(cloud_filtered->points[*point]);}

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = false;
        cont_nan_cluster->width = cluster->points.size();
        cont_nan_cluster->height = 1;
        cont_nan_cluster->is_dense = true;

           //保存聚类的结果
        if (cluster->points.size() <= 0)
            break;
        std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
        std::string fileName = "cluster" + boost::to_string(currentClusterNum) + ".pcd";
        pcl::io::savePCDFileASCII(fileName, *cluster);
        if (nan_cluster->points.size() != 0)
            nan_cluster->width = nan_cluster->points.size();
            nan_cluster->height = 1;
            nan_cluster->is_dense = false;
            pcl::io::savePCDFileASCII("nan_cluster.pcd", *nan_cluster);
        currentClusterNum++;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered1 (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::io::loadPCDFile("cluster1.pcd", *filtered);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZRGB>);  
    pcl::PassThrough<pcl::PointXYZRGB> pass1;
    pass1.setInputCloud (filtered);
    pass1.setFilterFieldName ("x");
    pass1.setFilterLimits (0.3, 1.5);
    //pass.setFilterLimitsNegative (true);
    pass1.filter (*cloud_filtered1);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB>);  
    pass1.setInputCloud (cloud_filtered1);
    pass1.setFilterFieldName ("y");
    pass1.setFilterLimits (-0.3, 0.3);
    //pass.setFilterLimitsNegative (true);
    pass1.filter (*cloud_filtered2);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outlier_filtered1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> Static1;   //创建滤波器对象
    Static1.setInputCloud (cloud_filtered2);                              //设置待滤波的点云
    Static1.setMeanK (100);                                         //设置在进行统计时考虑查询点临近点数
    Static1.setStddevMulThresh (1.0);                              //设置判断是否为离群点的阀值
    Static1.filter(*outlier_filtered1);

    pcl::io::savePCDFile("extract_object.pcd", *outlier_filtered1);

    // viewer.showCloud(cloud_filtered2);
    // while (!viewer.wasStopped ())
    // {
    // }
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (outlier_filtered1);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.005); // 2cm
    ec.setMinClusterSize (500);
    ec.setMaxClusterSize (8000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (outlier_filtered1);
    ec.extract (cluster_indices); 

    int j = 0;  
    float colors[6][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);  
    pcl::copyPointCloud(*outlier_filtered1, *cloud_cluster);  

    //due to save cluster
    int obj_num = cluster_indices.size();
    // sauces_all.resize(obj_num);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)  
    {  
        //

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {  
            //
            // PointT depth_pt = cloud_filtered->points[*pit];

            cloud_cluster->points[*pit].r = colors[j%6][0];  
            cloud_cluster->points[*pit].g = colors[j%6][1];  
            cloud_cluster->points[*pit].b = colors[j%6][2];  
            //
        }   

        //
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;  
        // std::stringstream ss;  
        // ss << "cloud_cluster_" << j << ".pcd";  
        // writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
        j++;  
    }  

    viewer.showCloud(cloud_cluster);
    while (!viewer.wasStopped ())
    {
    }

}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "show_cluster_object");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/zed/zed_node/point_cloud/cloud_registered", 1, cloud_cb);
    printf("test1\n");

    ros::spin ();
}
