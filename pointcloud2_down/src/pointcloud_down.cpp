#include <typeinfo>
#include "boost/range.hpp"
#include <ros/ros.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <string>

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h> 
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/PCLPointCloud2.h>

#include <stdio.h>
#include <iostream>
#include <vector>
#include <ctime>

#include <string.h>


/* 提示: 变换矩阵工作原理 :
     *        |-------> 变换矩阵列
     *  | 1 0 0 x |  \
     *  | 0 1 0 y |   }-> 左边是一个3阶的单位阵(无旋转)
     *  | 0 0 1 z |  /
     *  | 0 0 0 1 |    -> 这一行用不到 (这一行保持 0,0,0,1)

    /*  方法二 #2: 使用 Affine3f
     *  这种方法简单，不易出错
     */

typedef pcl::PointXYZRGB PointT;
std::string topic_pub_pcl="pcl_output";
std::string topic_sub_pcl="cloud_dpc";
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PCLPointCloud2 cloud;
sensor_msgs::PointCloud2 output;
ros::Publisher pcl_pub;  
ros::Subscriber sub_pcl;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);//降采样后的点云
Eigen::Affine3d transform_2 = Eigen::Affine3d::Identity();//坐标变换的矩阵
pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr kdtreeCloude;
std::vector<int> pointSearchInd;
std::vector<double> pointSearchSqDis;

double x_trans=0.0;
double y_trans=0.0;
double z_trans=0.52;
double theta_x = -M_PI/2;
double theta_y = 0;
double theta_z = -M_PI/2;

float leftSize = 0.03f;//体素大小
using namespace std;

void subPclCallback(const sensor_msgs::PointCloud2 &cloud_pcl){
	/**
	 * @brief 降采样并坐标转化
	 * @param cloud_pcl 接收的点云消息
	*/

	//降采样
    
	pcl::fromROSMsg(cloud_pcl,*cloud2);
    // kdtreeCloude.reset(new pcl::KdTreeFLANN<PointType>());
    // kdtreeCornerLast->setInputCloud(cloud2);
    // kdtreeCloude->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud2);
	sor.setLeafSize (leftSize, leftSize, leftSize);
	sor.filter(*cloud_filtered);
	//坐标变换
    pcl::PointCloud<PointT>::Ptr transformed_cloud (new pcl::PointCloud<PointT> ());
    pcl::transformPointCloud (*cloud_filtered, *transformed_cloud, transform_2);
	pcl::toROSMsg(*transformed_cloud, output); 
    output.header.frame_id = cloud_pcl.header.frame_id;
	output.header.stamp = cloud_pcl.header.stamp; 
    pcl_pub.publish(output);
}

int main (int argc, char **argv) 
{ 

    ros::init (argc, argv, "cloud_down"); 
    ros::NodeHandle nh; //ros句柄
    ros::NodeHandle nh_("~");
	nh_.getParam("topicPubPcl", topic_pub_pcl);
    nh_.getParam("topicSubPcl", topic_sub_pcl);
    nh_.getParam("leftSize", leftSize);
    nh_.getParam("x_trans", x_trans);
    nh_.getParam("y_trans", y_trans);
    nh_.getParam("z_trans", z_trans);
    nh_.getParam("theta_z", theta_z);
    nh_.getParam("theta_x", theta_x);
    nh_.getParam("theta_y", theta_y);
    cout<<theta_z<<endl;

    transform_2.translation() << x_trans, y_trans, z_trans;
    transform_2.rotate (Eigen::AngleAxisd (theta_z, Eigen::Vector3d::UnitZ()));
    transform_2.rotate (Eigen::AngleAxisd (theta_x, Eigen::Vector3d::UnitX()));
    transform_2.rotate (Eigen::AngleAxisd (theta_y, Eigen::Vector3d::UnitY()));
	sub_pcl = nh.subscribe(topic_sub_pcl, 10, &subPclCallback);
	pcl_pub = nh.advertise<sensor_msgs::PointCloud2> (topic_pub_pcl, 1);  
	
	ros::spin();
    return 0;
}
