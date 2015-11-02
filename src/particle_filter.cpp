#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseArray.h"
#include "tf/transform_datatypes.h"
#include "localization/Map_message.h"
#include "localization/Distance_message.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>

#define PI 3.14159265
#define VEL_SPREAD 0.1
#define ROT_SPREAD 0.1
#define NUM_PARTICLES 10000
#define NUM_OBSERVATIONS 6
#define SIGMA 0.1
#define NUM_WALLS 17
#define XMIN 0.0
#define XMAX 3.65
#define YMIN 0.0
#define YMAX 3.65

//Random number tror jag, testa : number = (double)std::rand() / (double)(std::RAND_MAX)

//TODO:switch around 1st and second array index in some of these!
ros::Publisher particles_pub;
ros::Publisher test_pub;
ros::Subscriber odom_sub;
ros::Subscriber obs_sub;
int global_index = 0;
double particles[3][NUM_PARTICLES]; //x,y,theta
double temp_particles[3][NUM_PARTICLES]; // x,y,theta
double probabilities[NUM_PARTICLES];
double predicted_observaions[NUM_OBSERVATIONS];
double observations[NUM_OBSERVATIONS];
double walls[NUM_WALLS][4];
double sensor_positions[6][3] = {
    {0.1,0.13,1.5707},
    {-0.1,0.13,1.5707},
    {0.1,-0.13,-1.5707},
    {-0.1,-0.13,-1.507},
    {0.12,-0.13,0.0},
    {0.12,0.13,0.0}
};
bool has_map = false;
bool has_measurements = false;
bool has_odom = false;


double normal_distribution_probabilitiy(double mean,double value){
    return (1.0/(SIGMA*sqrt(2.0*PI)))*exp(-((value-mean)*(value-mean))/(2.0*SIGMA*SIGMA));
}

void init_known_pos(double x, double y, double th){
    for(int j=0;j<NUM_PARTICLES;j++){
        particles[0][j] = x;
        particles[1][j] = y;
        particles[2][j] = th;
    }
}

void init_un_known_pos(double xmin,double ymin,double xmax,double ymax){
    for(int j=0;j<NUM_PARTICLES;j++){
        particles[0][j] = xmin + (xmax-xmin)*((double)std::rand() / (double)RAND_MAX);
        particles[1][j] = ymin + (ymax-ymin)*((double)std::rand() / (double)RAND_MAX);
        particles[2][j] = PI*2.0*((double)std::rand() / (double)RAND_MAX);
    }
}

void update(double v_vel, double rot_vel){

    for(int i=0;i<NUM_PARTICLES;i++){
        //TODO: fix zis
        particles[2][i] += rot_vel*ROT_SPREAD*((double)std::rand() / (double)RAND_MAX) - ROT_SPREAD/2.0 + ROT_SPREAD*((double)std::rand() / (double)RAND_MAX);
        double rand_vel = ((double)std::rand() / (double)RAND_MAX);
        particles[0][i] += v_vel*VEL_SPREAD*rand_vel*cos(particles[2][i]) - VEL_SPREAD/2.0 + VEL_SPREAD*((double)std::rand() / (double)RAND_MAX);
        particles[1][i] += v_vel*VEL_SPREAD*rand_vel*sin(particles[2][i]) - VEL_SPREAD/2.0 + VEL_SPREAD*((double)std::rand() / (double)RAND_MAX);
        if (particles[0][i]<XMIN) particles[0][i] = XMIN;
        else if (particles[0][i]>XMAX) particles[0][i] = XMAX;
        if (particles[1][i]<YMIN) particles[1][i] = YMIN;
        else if (particles[1][i]>YMAX) particles[1][i] = YMAX;
    }

/*
    for(int i=0;i<NUM_PARTICLES;i++){
        particles[2][i] += 5*((double)std::rand() / (double)RAND_MAX);
        particles[0][i] += -0.05 + 0.1*((double)std::rand() / (double)RAND_MAX);
        particles[1][i] += -0.05 + 0.1*((double)std::rand() / (double)RAND_MAX);
    }
*/
}

double dist(double x1,double y1,double x2,double y2){
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}
//Calculates distance from a point on a line to the intersection of that line with another line
//first four points are the first line. First two points marks the position to measure distance from.
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

visualization_msgs::Marker getMarker(double x,double y,double th,double dist){

    visualization_msgs::Marker marker;
    geometry_msgs::Point start_point;
    geometry_msgs::Point intersection_point;

    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.id = global_index;
    marker.scale.x = 0.01;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.1*(double)global_index;
    marker.color.b = 0.1*(double)global_index;


    start_point.x = x;
    start_point.y = y;
    start_point.z = 0.09;


    intersection_point.x = x + dist*cos(th);
    intersection_point.y = y + dist*sin(th);
    intersection_point.z = 0.09;


    //std::cout << x << " "<< y << " " << intersection_point.x << " "<<intersection_point.y << std::endl;

    marker.points.push_back(start_point);
    marker.points.push_back(intersection_point);



    return marker;

}

//puts the predicted observations for the particle in predicted_observations[].
void get_predicted_observations(double px,double py, double th){
    //visualization_msgs::MarkerArray marker_array;
    double x,y,theta,x2,y2,distance,shortest;
    for (int i=0;i<NUM_OBSERVATIONS;i++){
        x = px + sensor_positions[i][0];
        y = py + sensor_positions[i][1];
        theta = th + sensor_positions[i][2];
        x2 = x + 100*cos(theta);
        y2 = y + 100*sin(theta);
        shortest = 100.0;
        for(int j=0;j<NUM_WALLS;j++){
            if(get_intersection_distance(x,y,x2,y2,walls[j][0],walls[j][1],walls[j][2],walls[j][3],&distance)){
                if (distance<shortest){
                    shortest = distance;
                }
            }
        }
        //std::cout << x << " "<< y << " "<< theta << std::endl;
       // marker_array.markers.push_back(getMarker(x,y,theta,shortest));
       // std::cout << shortest << " ";

       predicted_observaions[i] = shortest;
       global_index = i+1;

    }
    //test_pub.publish(marker_array);
    //std::cout << std::endl;
}

void get_probabilities(){
    for (int i=0;i<NUM_PARTICLES;i++){
        double prob = 0.0;
        get_predicted_observations(particles[0][i],particles[1][i],particles[2][i]);
        for(int j=0;j<NUM_OBSERVATIONS;j++){

            //TODO: this could be optimized a lot! It could be a good idea to not have an if here but rather let the loop be decided outside. Don't know how to do this though.
            if(observations[j]>=0.01){
                prob+=normal_distribution_probabilitiy(predicted_observaions[j],observations[j]);
            }
            else{

            }
        }
        //std::cout << prob << std::endl;
        probabilities[i] = prob;
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
    poses.header.frame_id = "/map";
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



void get_observations(const localization::Distance_message::ConstPtr& msg){
    observations[0] = msg->d1;
    observations[1] = msg->d2;
    observations[2] = msg->d3;
    observations[3] = msg->d4;
    observations[4] = msg->d5;
    observations[5] = msg->d6;
    has_measurements = true;
}


int main(int argc,char **argv){
    //init_known_pos(0.0, 0.0, 0.0);
    init_un_known_pos(0.0, 0.0, 3.5, 3.5);
    ros::init(argc,argv,"particle_filter");
    ros::NodeHandle n;
    particles_pub = n.advertise<geometry_msgs::PoseArray>("/particles", 100);
    test_pub = n.advertise<visualization_msgs::MarkerArray>("/test", 100);
    //odom_sub = n.subscribe("/odom",100,odom_callback);
    obs_sub = n.subscribe("/ir_measurements", 100, get_observations);

    ros::Publisher map_query_publisher = n.advertise<std_msgs::Bool>("/map_reader/query", 100);
    ros::Subscriber map_subscriber = n.subscribe("/map_reader/map", 100, read_map);


    while (!has_map && ros::ok()){
        std_msgs::Bool bool_msg;
        bool_msg.data = true;
        map_query_publisher.publish(bool_msg);
        ros::spinOnce();
    }
    //observations[0] = 0.5;
    //observations[1] = 0.5;
    //observations[2] = 0.5;
    //observations[3] = 0.5;
    //observations[4] = 0.5;
    //observations[5] = 0.5;
    //std::cout << normal_distribution_probabilitiy(1.0,0.9) << std::endl;
    //std::cout << normal_distribution_probabilitiy(0.9,1.0) << std::endl;
    //std::cout << normal_distribution_probabilitiy(0.1,0.1) << std::endl;
    //std::cout << normal_distribution_probabilitiy(1.0,1.0) << std::endl;
    //std::cout << normal_distribution_probabilitiy(6.0,6.0) << std::endl;

    std::cout << "Started particle filtering" << std::endl;

    has_odom = true;
    while (ros::ok()){
        if (has_measurements && has_odom){
            std::cout << "lall"<<std::endl;
            update(0.05,0.0);
            get_probabilities();
            resample();
            draw();
            has_measurements = false;
            //has_odom = false;
        }
        ros::spinOnce();
    }
}
