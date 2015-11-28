#include "ros/ros.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "localization/Distance_message.h"
#include <math.h>
#include <iostream>


#define NUM_MEASUREMENTS 11
#define MIDDLE 5


double s_1 [NUM_MEASUREMENTS];
double s_2 [NUM_MEASUREMENTS];
double s_3 [NUM_MEASUREMENTS];
double s_4 [NUM_MEASUREMENTS];
double s_5 [NUM_MEASUREMENTS];
double s_6 [NUM_MEASUREMENTS];
int array_pos = 0;

bool publishing = false;

ros::Publisher distance_pub;



double translation(double a,double b, double c, double d, double x){
    return a*exp(b*x) + c*exp(d*x);
}

double long1_dist_conv(double x){
    if(x<100.0) return 0.0;
    else{return translation(2.341,-0.01889,0.3807,-0.002729,x);}
}

double long2_dist_conv(double x){
    if(x<100.0) return 0.0;
    else{return translation(6.482,-0.03276,0.6919,-0.004511,x);}
}

double short_dist_conv(double x){
    if(x<50.0) return 0.0;

    else{return translation(0.6579,-0.02849,0.2235,-0.003499,x);}

}

double sensor1_conv(double x){
    if(x<60.0) return 0.0;

    else{return translation(60.24,-0.02238,28.4,-0.003959,x);}
}

double sensor2_conv(double x){
    if(x<110.0) return 0.0;

    else{return translation(154.9,-0.0168,40.16,-0.00275,x);}
}

double sensor3_conv(double x){
    if(x<60.0) return 0.0;

    else{return translation(54.801,-0.01931,22.53,-0.003468,x);}
}

double sensor4_conv(double x){
    if(x<60.0) return 0.0;

    else{return translation(57.16,-0.01497,8.454,-1.819e-5,x);}
}

double sensor5_conv(double x){
    if(x<60.0) return 0.0;

    else{return translation(75.19,-0.01282,6.649,0.0006013,x);}
}

double sensor6_conv(double x){

    if(x<120.0) return 0.0;

    else{return translation(187.2,-0.01927,39.23,-0.002973,x);}
}

double get_median(double the_vector [NUM_MEASUREMENTS]){

    double values[NUM_MEASUREMENTS];
    for (int i=0;i<NUM_MEASUREMENTS;i++){
        values[i] = the_vector[i];
    }

    double smallest = 100000.0;
    int last_pos = -1;

    //Get smallest
    for (int k=0; k<MIDDLE; k++){
        smallest = 100000.0;
        for (int i=0;i<NUM_MEASUREMENTS;i++){
            if (values[i]<smallest){
                smallest = values[i];
                last_pos = i;
            }
        }
        values[last_pos] = 100000.0;
    }
    return smallest;
}


void adcallback(const ras_arduino_msgs::ADConverter::ConstPtr& msg){

    s_1[array_pos] = (double)msg->ch1;
    s_2[array_pos] = (double)msg->ch4;
    s_3[array_pos] = (double)msg->ch3;
    s_4[array_pos] = (double)msg->ch8;
    s_5[array_pos] = (double)msg->ch7;
    s_6[array_pos] = (double)msg->ch2;


    array_pos += 1;
    if (array_pos >= NUM_MEASUREMENTS){
        array_pos = 0;
        publishing = true;
    }
    if (!publishing){
        return;
    }

    localization::Distance_message out;

    out.d1 = sensor1_conv(get_median(s_1)) * 0.01;
    out.d2 = sensor2_conv(get_median(s_2)) * 0.01;
    out.d3 = sensor3_conv(get_median(s_3)) * 0.01;
    out.d4 = sensor4_conv(get_median(s_4)) * 0.01;
    out.d5 = sensor5_conv(get_median(s_5)) * 0.01;
    out.d6 = sensor6_conv(get_median(s_6)) * 0.01;
    distance_pub.publish(out);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance_converter");
    ros::NodeHandle n;
    distance_pub = n.advertise<localization::Distance_message>("/ir_measurements", 100);
    ros::Subscriber adc_sub = n.subscribe("/arduino/adc", 100, adcallback);
    ros::spin();
    return 0;
}
