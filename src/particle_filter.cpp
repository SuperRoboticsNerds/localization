#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseArray.h"
#include "tf/transform_datatypes.h"
#include "localization/Map_message.h"
#include <iostream>
#include <stdlib.h>
#include <math.h>

#define PI 3.14159265
#define VEL_SPREAD 0.1
#define ROT_SPREAD 0.1
#define NUM_PARTICLES 5000
#define NUM_OBSERVATIONS 6
#define SIGMA 0.1
#define NUM_WALLS 17

//Random number tror jag, testa : number = (double)std::rand() / (double)(std::RAND_MAX)

//TODO:switch around 1st and second array index in some of these!
ros::Publisher particles_pub;
ros::Subscriber odom_sub;
double particles[3][NUM_PARTICLES];
double temp_particles[3][NUM_PARTICLES]; // x,y,theta
double probabilities[NUM_PARTICLES];
double observations[NUM_OBSERVATIONS];
double walls[NUM_WALLS][4];
bool has_map = false;



void odom_callback(){}

double normal_distribution_probabilitiy(double mean,double value){
    return (1.0/(SIGMA*sqrt(2.0*PI)))*exp(-(value-mean)*(value-mean)/(2.0*SIGMA*SIGMA));
}

void init_known_pos(double x, double y, double th){
    for(int j=0;j<NUM_PARTICLES;j++){
        particles[0][j] = x;
        particles[1][j] = y;
        particles[2][j] = th;
    }
}

void update(double v_vel, double rot_vel){
    for(int i=0;i<NUM_PARTICLES;i++){
        particles[2][i] += rot_vel*ROT_SPREAD*((double)std::rand() / (double)RAND_MAX);
        particles[0][i] += v_vel*VEL_SPREAD*((double)std::rand() / (double)RAND_MAX)*cos(particles[2][i]);
        particles[1][i] += v_vel*VEL_SPREAD*((double)std::rand() / (double)RAND_MAX)*sin(particles[2][i]);
    }
}

double dist(double x1,double y1,double x2,double y2){
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}


void get_probabilities(){
    for(int i=0;i<NUM_PARTICLES;i++){
        double prob = 0.0;
        for(int j=0;NUM_WALLS;j++){


        }



    }
}



void get_probabilities_test(){
    double center_x = 0.0,center_y = 0.0;
    for (int i=0;i<NUM_PARTICLES;i++){
        double distance = dist(particles[0][i],particles[1][i],center_x,center_y);
        probabilities[i] = normal_distribution_probabilitiy(1.0,distance);
    }
}

void resample(){

    //Normalize and make cumsum
    double sum = 0.0;
    //Calculate normalizer
    for (int i=0;i<NUM_PARTICLES;i++){
        sum+=probabilities[i];
    }
    //Cacĺculate cumsum
    double cumsum = 0.0;
    for (int i=0;i<NUM_PARTICLES;i++){
        cumsum += probabilities[i]/sum;
        probabilities[i] = cumsum;
    }
    //Create temporary particles
    for(int i=0;i<NUM_PARTICLES;i++){
        temp_particles[0][i] = particles[0][i];
        temp_particles[1][i] = particles[1][i];
        temp_particles[2][i] = particles[2][i];
    }
    //Resample!
    for (int i=0;i<NUM_PARTICLES;i++){
        double temp = (double)std::rand() / (double)RAND_MAX;


        for(int pos=0;pos<NUM_PARTICLES;pos++){
            //TODO: this if-case inside of a loop is not a good idea for efficiency :(
            if (temp<probabilities[pos]){

                particles[0][i] = temp_particles[0][pos];
                particles[1][i] = temp_particles[1][pos];
                particles[2][i] = temp_particles[2][pos];
                break;
            }
        }
    }
}


void particle_filter(){
    update(0,0);
    get_probabilities();
    resample();
    //if position update and ir update, do the rest.


}


void draw(){
    geometry_msgs::PoseArray poses;
    poses.header.frame_id = "/base_link";
    poses.header.stamp = ros::Time::now();
    for (int i=0;i<NUM_PARTICLES;i++){
        geometry_msgs::Pose pose;
        pose.position.x = particles[0][i];
        pose.position.y = particles[1][i];
        double rot = particles[2][i];
        pose.orientation = tf::createQuaternionMsgFromYaw(rot);
        poses.poses.push_back(pose);
    }

    particles_pub.publish(poses);
}

void read_map(const localization::Map_message::ConstPtr& msg){

    for(int i=0;i<NUM_WALLS;i++){
        walls[i][0] = msg->points[i*4];
        walls[i][1] = msg->points[i*4+1];
        walls[i][2] = msg->points[i*4+2];
        walls[i][3] = msg->points[i*4+3];
    }
    has_map = true;

}

//Calculates distance from a point on a line to the intersection of that line with another line
bool get_intersection_distance(double p0_x, double p0_y, double p1_x, double p1_y,
    double p2_x, double p2_y, double p3_x, double p3_y, double * distance)
{
    double i_x,i_y;

    double s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

    double s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
    {
        // Collision detected
        i_x = p0_x + (t * s1_x);
        i_y = p0_y + (t * s1_y);
        *distance = sqrt((i_x-p0_x)*(i_x-p0_x) + (i_y-p0_y)*(i_y-p0_y));
        return true;
    }

    return false; // No collision
}


int main(int argc,char **argv){
    init_known_pos (0,0,0);
    ros::init(argc,argv,"particle_filter");
    ros::NodeHandle n;
    particles_pub = n.advertise<geometry_msgs::PoseArray>("/particles",100);
    //odom_sub = n.subscribe("/odom",100,odom_callback);

    ros::Publisher map_query_publisher = n.advertise<std_msgs::Bool>("/map_reader/query",100);
    ros::Subscriber map_subscriber = n.subscribe("/map_reader/map",100,read_map);
    while (!has_map && ros::ok()){
        std_msgs::Bool bool_msg;
        bool_msg.data = true;
        map_query_publisher.publish(bool_msg);
        ros::spinOnce();
    }


    while (ros::ok()){
        update(1.0,1.0);
        get_probabilities_test();
        resample();
        draw();
        ros::spinOnce();
    }

}
