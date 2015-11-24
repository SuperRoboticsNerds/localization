#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseArray.h"
#include "tf/transform_datatypes.h"
#include "localization/Map_message.h"
#include "localization/Distance_message.h"
#include "geometry_msgs/Twist.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>

#define PI 3.14159265
#define VEL_SPREAD 0.3
#define ROT_SPREAD 0.3
#define NUM_PARTICLES 10000
#define NUM_OBSERVATIONS 4
#define SIGMA 0.3
#define NUM_WALLS 17
#define XMIN 0.0
#define XMAX 3.65
#define YMIN 0.0
#define YMAX 3.65

//Random number tror jag, testa : number = (double)std::rand() / (double)(std::RAND_MAX)
//TODO: only update when the robot actually has changed its position. (or update like 5 times, then stop updating)

//TODO:switch around indecies in particles and temp_particles. There are 46 occurences of this problem. Should be irradicated!!!!!!!
//Also performance could go up OR down if this is done - careful!!!
//The sensors should be four short-range to the sides and long-range ones pointing forward and back!!!

ros::Publisher particles_pub;
ros::Publisher test_pub;
ros::Subscriber odom_sub;
ros::Subscriber vel_sub;
ros::Subscriber obs_sub;
int global_index = 0;
double lastTime;
double prevx,prevy,prevth,currx,curry,currth;
double particles[3][NUM_PARTICLES]; //x,y,theta
double temp_particles[3][NUM_PARTICLES]; // x,y,theta
double probabilities[NUM_PARTICLES];
double predicted_observaions[NUM_OBSERVATIONS];
double observations[NUM_OBSERVATIONS];
double walls[NUM_WALLS][4];

//Define the positions and orientations of the sensors here. {X,Y,THETA} THETA = 0 is straight forward.
double sensor_positions[NUM_OBSERVATIONS][3] = {
    //THIS IS FOR THE SIMULATION
    /*
    {0.1,0.13,1.5707},
    {-0.1,0.13,1.5707},
    {0.1,-0.13,-1.5707},
    {-0.1,-0.13,-1.507},
    {0.12,-0.13,0.0},
    {0.12,0.13,0.0}};
    */
    {5.0,4.0,PI/2.0},
    {5.0,1.5,0},
    {4.0,-4.0,-PI/2.0},
    {-2.0,1.5,PI/2.0},
    //{-2.5,-2.0,-PI/2.0},
    //{-4.0,-1.5,PI},

};
    /*
        //{X,Y, Rotation Theta} Where pi/2 = 1.5707
        {0.075,-0.04,3.1416},      // Short Range Left Front
        {-0.0745,-0.04,3.1416},     // Short Range Left Back
        {0.0745,0.04,0},       // Short Range Right Front
        {-0.07,0.04,0},        // Short Range Right Back
        {0.0225,0,1.5707},           // Long Range Front  //or distance_sensor_forward_right_link
        {0.0225,0,1.5707}
    */


bool has_map = false;
bool has_measurements = false;
bool has_odom = false;
static double x_linear_vel=0.0;
static double z_angular_vel =0.0;


double normal_distribution_probabilitiy(double mean,double value){
    return (1.0/(SIGMA*sqrt(2.0*PI)))*exp(-((value-mean)*(value-mean))/(2.0*SIGMA*SIGMA));
}

//Initialize all particles at the same position(will soon be spread out)
void init_known_pos(double x, double y, double th){
    for(int j=0;j<NUM_PARTICLES;j++){
        particles[0][j] = x;
        particles[1][j] = y;
        particles[2][j] = th;
    }
}

//Initialize particles within a box, thetas random
void init_un_known_pos(double xmin,double ymin,double xmax,double ymax){
    for(int j=0;j<NUM_PARTICLES;j++){
        particles[0][j] = xmin + (xmax-xmin)*((double)std::rand() / (double)RAND_MAX);
        particles[1][j] = ymin + (ymax-ymin)*((double)std::rand() / (double)RAND_MAX);
        particles[2][j] = PI*2.0*((double)std::rand() / (double)RAND_MAX);
    }
}

int sign(double val){
    return (0 < val) - (val < 0);
}

//Updates the positions of the particles depending on what has happened in the odometry
void update(){
    double rot_vel = (currth-prevth);
    double xdiff = currx-prevx;
    double ydiff = curry-prevy;
    double v_vel = sqrt(xdiff*xdiff + ydiff*ydiff);
    if(sign(xdiff)!=sign(cos(currth))){
        v_vel = -v_vel;
    }
    for(int i=0;i<NUM_PARTICLES;i++){
        //TODO: fix zis
        particles[2][i] += rot_vel - ROT_SPREAD*rot_vel/2.0 + ROT_SPREAD*rot_vel*((double)std::rand() / (double)RAND_MAX);
        double rand_vel = ((double)std::rand() / (double)RAND_MAX);
        particles[0][i] += v_vel*cos(particles[2][i]) - VEL_SPREAD*v_vel*cos(particles[2][i])/2.0 + v_vel*VEL_SPREAD*rand_vel*cos(particles[2][i]);
        particles[1][i] += v_vel*sin(particles[2][i]) - VEL_SPREAD*v_vel*sin(particles[2][i])/2.0 + v_vel*VEL_SPREAD*rand_vel*sin(particles[2][i]);
        if (particles[0][i]<XMIN) particles[0][i] = XMIN;
        else if (particles[0][i]>XMAX) particles[0][i] = XMAX;
        if (particles[1][i]<YMIN) particles[1][i] = YMIN;
        else if (particles[1][i]>YMAX) particles[1][i] = YMAX;
        //std::cout << particles[0][i] << " " << particles[1][i] << std::endl;

    }


    prevx = currx;
    prevy = curry;
    prevth = currth;
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
//Share desktop to get this to run faster over network
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

//Assigns probabilities to the particles depending on the map
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

//Resamples the particles
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

//Used to visualize the particles
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

//Called once to read the map published from another ros node
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

//Updates the odometry
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
    currx = msg->pose.pose.position.x;
    curry = msg->pose.pose.position.y;
    //TODO: this is retarted. This is because of how it is set up on the kobuki simulation and we can do better irr (in real robot)
    double roll, pitch, yaw;
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    tf::Quaternion q(qx,qy,qz,qw);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    //std::cout << yaw << std::endl;
    currth = yaw;
    has_odom = true;
}
void vel_func(const geometry_msgs::Twist::ConstPtr& twist_msg){

    x_linear_vel = twist_msg->linear.x;
    z_angular_vel = twist_msg->angular.z;
//ROS_INFO("%f,%f",x_linear_vel, z_angular_vel);
//std::cout << x_linear_vel<<std::endl;
}


int main(int argc,char **argv){
    //init_known_pos(0.0, 0.0, 0.0);
    init_un_known_pos(0.0, 0.0, 3.65, 3.65);
    ros::init(argc,argv,"particle_filter");
    ros::NodeHandle n;
    particles_pub = n.advertise<geometry_msgs::PoseArray>("/particles", 100);
    test_pub = n.advertise<visualization_msgs::MarkerArray>("/test", 100);
    odom_sub = n.subscribe("/odom",100,odom_callback);
    vel_sub = n.subscribe("/motor_controller/twist",100, vel_func);
    obs_sub = n.subscribe("/ir_measurements", 100, get_observations);

    ros::Publisher map_query_publisher = n.advertise<std_msgs::Bool>("/map_reader/query", 100);
    ros::Subscriber map_subscriber = n.subscribe("/map_reader/map", 100, read_map);


    while (!has_map && ros::ok()){
        std_msgs::Bool bool_msg;
        bool_msg.data = true;
        map_query_publisher.publish(bool_msg);
        ros::spinOnce();
    }


    std::cout << "Started particle filtering" << std::endl;

    //~~~~~THE PARTICLE FILTER ALGORITHM~~~~~\\

    has_odom = true; //TODO: I forgot why I set this to true here...
    while (ros::ok()){
        //std::cout << x_linear_vel<<std::endl;
        if (x_linear_vel<= 1.0e-5){
            x_linear_vel=0;
        }
        if (z_angular_vel<= 1.0e-5){
            z_angular_vel=0;
        }
        if ((x_linear_vel!=0 || z_angular_vel!=0 )&& (has_measurements && has_odom)){

            // if the robot moves, update the particle filter
            std::cout << "calculating..."<<std::endl;
            update();
            get_probabilities();
            resample();
            draw();
            has_measurements = false;
            has_odom = false;
       }

        ros::spinOnce();
    }
}
