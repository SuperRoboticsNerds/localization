#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <localization/Depth_Ranges.h>
#include <stdlib.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

const int NUM_MEASUREMENTS = 10;
const int SAFE_ZONE = 10;
const int ROW_TO_LOOK_AT = 20;

//TODO: it is possible that this algorithm chooses two of the same measurements. But I think that is fine since there is a low probability and it probably doens't matter that much



ros::Publisher depth_pub;

void callback(const PointCloud::ConstPtr& msg)
{

    localization::Depth_Ranges depth_msg;
    depth_msg.NUM_MEASUREMENTS = NUM_MEASUREMENTS;

    int width = msg->width;
    int heigth = msg->height;


    for(int i=0;i<NUM_MEASUREMENTS;i++){
        int pixel = rand() % (width-SAFE_ZONE) + SAFE_ZONE;
        pcl::PointXYZ pt = msg->points[width*ROW_TO_LOOK_AT + pixel];

        printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_ranges_publisher");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/depth/points", 1, callback);
  depth_pub = nh.advertise<localization::Depth_Ranges>("/localization/depth_ranges",1);
  ros::spin();
}
