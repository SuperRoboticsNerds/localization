#include "ros/ros.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "localization/Distance_message.h"

ros::Publisher distance_pub;

double long_dist_conv(double x){
    if(x<15.0) return 0.0;
    // return 0.000003188*x*x - 0.0034046669*x + 1.02568224;
    return   0.000000109* pow(x, 3) -0.0000114150226132116*pow(x, 2) -0.0038 *x + 0.944;

}



double short_dist_conv(double x){
    if(x<15.0) return 0.0;
    return  0.0000558 * pow(x, 3) - 0.00704347775167042 * x*x  + 0.553495160550501;
    // return 0.000003188*x*x - 0.0034046669*x + 1.02568224;

}

void adcallback(const ras_arduino_msgs::ADConverter::ConstPtr& msg){

    localization::Distance_message out;
    out.d1 = short_dist_conv((double)msg->ch1);
    out.d2 = short_dist_conv((double)msg->ch2);
    out.d3 = short_dist_conv((double)msg->ch3);
    out.d4 = short_dist_conv((double)msg->ch7);
    out.d5 = short_dist_conv((double)msg->ch8);
    out.d6 = long_dist_conv((double)msg->ch8);
    distance_pub.publish(out);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance_converter");
    ros::NodeHandle n;
    distance_pub = n.advertise<localization::Distance_message>("/ir_measurements", 1000);
    ros::Subscriber adc_sub = n.subscribe("/arduino/adc", 1000, adcallback);
    ros::spin();
    return 0;
}
