#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <localization/Depth_Ranges.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>
#include <math.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

const int NUM_MEASUREMENTS = 10;
const int SAFE_ZONE = 10;
const int ROW_TO_LOOK_AT = 20;
const float ANGLE_TO_FLOOR = 0.17453292519; //TODO: change this!!!

//TODO: it is possible that this algorithm chooses two of the same measurements. But I think that is fine since there is a low probability and it probably doens't matter that much



ros::Publisher depth_pub;
ros::Publisher lines_pub;


void callback(const PointCloud::ConstPtr& msg)
{

    localization::Depth_Ranges depth_msg;
    depth_msg.NUM_MEASUREMENTS = NUM_MEASUREMENTS;

    int width = msg->width;
    int heigth = msg->height;

    std::cout << "derpp" << std::endl;


    for(int i=0;i<NUM_MEASUREMENTS;i++){
        int pixel = rand() % (width-SAFE_ZONE) + SAFE_ZONE;

        std::cout << pixel << std::endl;

        pcl::PointXYZ pt = msg->points[width*heigth/5 + pixel];

        double z_distance = cos(ANGLE_TO_FLOOR)*pt.z;
        double distance = sqrt(pt.x*pt.x + z_distance*z_distance);
        double angle = atan2(pt.x,z_distance);

        //Check if nan
        if (distance != distance){
            depth_msg.angles.push_back(0.0);
            depth_msg.distances.push_back(0.0);
        }
        else{
            depth_msg.angles.push_back(angle);
            depth_msg.distances.push_back(distance);
        }
    }

    depth_pub.publish(depth_msg);

    //Publish distances, this is just to test!!!!
    visualization_msgs::MarkerArray marker_array;

    for(int i=0;i<NUM_MEASUREMENTS;i++){

        visualization_msgs::Marker marker;
        geometry_msgs::Point start_point;
        geometry_msgs::Point intersection_point;

        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.id = i;
        marker.scale.x = 0.01;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.1;
        marker.color.b = 1.0;

        start_point.x = 0;
        start_point.y = 0;
        start_point.z = 0.09;

        double temp_dist = depth_msg.distances[i];
        double temp_angle = depth_msg.angles[i];

        intersection_point.x = temp_dist*sin(temp_angle);
        intersection_point.y = temp_dist*cos(temp_angle);
        intersection_point.z = 0.09;

        marker.points.push_back(start_point);
        marker.points.push_back(intersection_point);

        marker_array.markers.push_back(marker);

    }

    lines_pub.publish(marker_array);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_ranges_publisher");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/depth/points", 1, callback);
  depth_pub = nh.advertise<localization::Depth_Ranges>("/localization/depth_ranges",1);
  lines_pub = nh.advertise<visualization_msgs::MarkerArray>("/depth_lines",1);
  ros::spin();
}
