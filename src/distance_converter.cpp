#include "ros/ros.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "localization/Distance_message.h"

ros::Publisher distance_pub;

double dist_conv(double x){
    if(x<5.0) return 0.0;
    return 0.000003188*x*x - 0.0034046669*x + 1.02568224;

}


void adcallback(const ras_arduino_msgs::ADConverter::ConstPtr& msg){

    localization::Distance_message out;
    out.d1 = dist_conv((double)msg->ch1);
    out.d2 = dist_conv((double)msg->ch2);
    out.d3 = dist_conv((double)msg->ch3);
    out.d4 = dist_conv((double)msg->ch4);
    out.d5 = dist_conv((double)msg->ch5);
    out.d6 = dist_conv((double)msg->ch6);
    distance_pub.publish(out);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance_converter");
    ros::NodeHandle n;
    distance_pub = n.advertise<localization::Distance_message>("/ir_measurements", 1000);
    ros::Subscriber adc_sub = n.subscribe("/kobuki/adc", 1000, adcallback);
    ros::spin();
    return 0;
}
