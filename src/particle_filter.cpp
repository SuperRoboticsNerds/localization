﻿#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseArray.h"
#include "tf/transform_datatypes.h"
#include "localization/Map_message.h"
#include "localization/Distance_message.h"
#include "localization/Position.h"
#include "geometry_msgs/Twist.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <motors/odometry.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <boost/lexical_cast.hpp>

#define PI 3.14159265

#define NUM_PARTICLES 10000
#define NUM_OBSERVATIONS 6
#define UPDATES_BEFORE_RESAMPLE 5
#define TIMES_TO_RESAMPLE 1
#define SIGMA 0.001
#define NUM_WALLS 100 //This is just because I don't know how many they are
#define XMIN 0.0
#define YMIN 0.0

//Random number tror jag, testa : number = (double)std::rand() / (double)(std::RAND_MAX)

//The sensors should be four short-range to the sides and long-range ones pointing forward and back!!!
localization::Position position;
ros::Publisher particles_pub;
ros::Publisher test_pub;
ros::Publisher position_pub;
ros::Publisher beams_pub;
ros::Publisher pos_marker_pub;
ros::Subscriber odom_sub;
ros::Subscriber vel_sub;
ros::Subscriber obs_sub;

int number_of_walls = 0;
int global_index = 0;
int observations_actually_observed = 0;
double safety_distance = 0.05; //if particle is closer to the walls than this they will bounce
double lastTime;
double forward_movement,rotation_movement;
double particles[NUM_PARTICLES][3]; //x,y,theta
double temp_particles[NUM_PARTICLES][3]; // x,y,theta
double probabilities[NUM_PARTICLES];
double predicted_observaions[NUM_OBSERVATIONS];
double observations[NUM_OBSERVATIONS];
double temp_observations[NUM_OBSERVATIONS];
double walls[NUM_WALLS][4];
double cumsum[NUM_PARTICLES];
double temp_probabilities[NUM_PARTICLES];



// Rosparam settings

double rot_spread = 0.0;
double vel_spread = 0.0;
double prob_sigma = 0.0;
int updates_before_resample = 0;
int times_to_resample = 0;
double maze_xmax = 0.0;
double maze_ymax = 0.0;

//Define the positions and orientations of the sensors here. {X,Y,THETA} THETA = 0 is straight forward.
double sensor_positions[NUM_OBSERVATIONS][3];

    /*
    //Erik's measurements (crude)

    {0.07,0.015,50.0*PI/180.0},
    {0.04,0.0,0.0},
    {0.07,-0.015,-60.0*PI/180.0},
    {0.0,-0.02,-PI/2.0},
    {0.0,0.02,PI/2.0},
    {-0.06,0.0, PI}
*/
/*
    //Mattias' measurements (ir sensor)
    {0.042,0.022,50.0*PI/180.0},
    {0.03,-0.009,0.0},
    {0.030,-0.032,-60.0*PI/180.0},
    {-0.031,-0.025,-PI/2.0},
    {-0.013,0.026,PI/2.0},
    {-0.09,0.017, PI}
*/
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


double normal_distribution_probabilitiy(double mean,double value){
    return (1.0/(SIGMA*sqrt(2.0*PI)))*exp(-((value-mean)*(value-mean))/(2.0*SIGMA*SIGMA));
}

//Initialize all particles at the same position(will soon be spread out)
void init_known_pos(double x, double y, double th){
    for(int j=0;j<NUM_PARTICLES;j++){
        particles[j][0] = x;
        particles[j][1] = y;
        particles[j][2] = th;
    }
}

//Initialize particles within a box, thetas random
void init_un_known_pos(double xmin,double ymin,double xmax,double ymax){
    for(int j=0;j<NUM_PARTICLES;j++){
        particles[j][0] = xmin + (xmax-xmin)*((double)std::rand() / (double)RAND_MAX);
        particles[j][1] = ymin + (ymax-ymin)*((double)std::rand() / (double)RAND_MAX);
        particles[j][2] = PI*2.0*((double)std::rand() / (double)RAND_MAX);
    }
}

int sign(double val){
    return (0 < val) - (val < 0);
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

//Updates the positions of the particles depending on what has happened in the odometry
void update(){

    double forward_while_update = forward_movement;
    double rotation_while_update = rotation_movement;

    forward_movement = 0.0;
    rotation_movement = 0.0;

    for(int i=0;i<NUM_PARTICLES;i++){

        particles[i][2] += rotation_while_update + rot_spread*rotation_while_update*(((double)std::rand() / (double)RAND_MAX)-0.5);
        double rand_vel = ((double)std::rand() / (double)RAND_MAX) - 0.5;


        particles[i][0] +=  forward_while_update*cos(particles[i][2]) + forward_while_update*vel_spread*rand_vel*cos(particles[i][2]);
        particles[i][1] +=  forward_while_update*sin(particles[i][2]) + forward_while_update*vel_spread*rand_vel*sin(particles[i][2]);

        //updated particles should never cross walls.
        /*
        for (int j=0;j<NUM_WALLS;j++){
            if(get_intersection_distance(particles[i][0],particles[i][1],newx,newy,walls[j][0],walls[j][1],walls[j][2],walls[j][3],&distance)){
                newx = particles[i][0] +  cos(particles[i][2]) * (distance - safety_distance);
                newy = particles[i][1] +  sin(particles[i][2]) * (distance - safety_distance);
                break; //Doesn't take into consideration that a line might cross two walls
            }
        }
        */
        if (particles[0][i]<XMIN) particles[0][i] = XMIN;
        else if (particles[0][i]>maze_xmax) particles[0][i] = maze_xmax;
        if (particles[1][i]<YMIN) particles[1][i] = YMIN;
        else if (particles[1][i]>maze_ymax) particles[1][i] = maze_ymax;

    }
}

void publishBeams(double px,double py,double th){
    visualization_msgs::MarkerArray marker_array;

    double x,y,theta,x2,y2,distance,shortest;

    for(int i=0;i<NUM_OBSERVATIONS;i++){

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

        theta = th + sensor_positions[i][2];
        x = px + sensor_positions[i][0]*cos(th) - sensor_positions[i][1]*sin(th);
        y = py + sensor_positions[i][0]*sin(th) + sensor_positions[i][1]*cos(th);

        x2 = x + 100.0*cos(theta);
        y2 = y + 100.0*sin(theta);
        shortest = 100.0;
        for(int j=0;j<number_of_walls;j++){
            if(get_intersection_distance(x,y,x2,y2,walls[j][0],walls[j][1],walls[j][2],walls[j][3],&distance)){
                if (distance<shortest){
                    shortest = distance;
                }
            }
        }

        start_point.x = x;
        start_point.y = y;
        start_point.z = 0.09;

        if(observations[i] >0.01){

            intersection_point.x = x + observations[i]*cos(theta);
            intersection_point.y = y + observations[i]*sin(theta);
            intersection_point.z = 0.09;
        }
        else{
            intersection_point.x = x;
            intersection_point.y = y;
            intersection_point.z = 0.09;

        }

        marker.points.push_back(start_point);
        marker.points.push_back(intersection_point);

        marker_array.markers.push_back(marker);

    }

    beams_pub.publish(marker_array);


}

//puts the predicted observations for the particle in predicted_observations[].
void get_predicted_observations(double px,double py, double th){
    //visualization_msgs::MarkerArray marker_array;
    double x,y,theta,x2,y2,distance,shortest;
    for (int i=0;i<NUM_OBSERVATIONS;i++){
        theta = th + sensor_positions[i][2];
        x = px + sensor_positions[i][0]*cos(th) - sensor_positions[i][1]*sin(th);
        y = py + sensor_positions[i][0]*sin(th) + sensor_positions[i][1]*cos(th);

        x2 = x + 100.0*cos(theta);
        y2 = y + 100.0*sin(theta);
        shortest = 100.0;
        for(int j=0;j<number_of_walls;j++){
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
        get_predicted_observations(particles[i][0],particles[i][1],particles[i][2]);
        for(int j=0;j<NUM_OBSERVATIONS;j++){

            //TODO: this could be optimized a lot! It could be a good idea to not have an if here but rather let the loop be decided outside. Don't know how to do this though.
            if(observations[j]>=0.01){ //The if-statement gets rid of shitty readings.
                prob+=normal_distribution_probabilitiy(predicted_observaions[j],observations[j]);
            }
            else{

            }
        }
        //std::cout << prob << std::endl;
        probabilities[i] = (prob + probabilities[i]) / 2.0;
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
        double the_sum = 0.0;
        for (int i=0;i<NUM_PARTICLES;i++){
            the_sum += probabilities[i]/sum;
            cumsum[i] = the_sum;
        }
        //Create temporary particles
        for(int i=0;i<NUM_PARTICLES;i++){
            temp_particles[i][0] = particles[i][0];
            temp_particles[i][1] = particles[i][1];
            temp_particles[i][2] = particles[i][2];
            temp_probabilities[i] = probabilities[i];
        }
        //Resample!
        for (int i=0;i<NUM_PARTICLES;i++){
            double temp = (double)std::rand() / (double)RAND_MAX;

            for(int pos=0;pos<NUM_PARTICLES;pos++){
                //TODO: this if-case inside of a loop is not a good idea for efficiency :(
                if (temp<cumsum[pos]){
                    particles[i][0] = temp_particles[pos][0];
                    particles[i][1] = temp_particles[pos][1];
                    particles[i][2] = temp_particles[pos][2];
                    probabilities[i] = temp_probabilities[i];
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
        pose.position.x = particles[i][0];
        pose.position.y = particles[i][1];
        double rot = particles[i][2];
        pose.orientation = tf::createQuaternionMsgFromYaw(rot);
        poses.poses.push_back(pose);
    }
    particles_pub.publish(poses);
}

//Called once to read the map published from another ros node
void read_map(const localization::Map_message::ConstPtr& msg){
    number_of_walls = msg->number_of_walls;
    for(int i=0;i<number_of_walls;i++){
        walls[i][0] = msg->points[i*4];
        walls[i][1] = msg->points[i*4+1];
        walls[i][2] = msg->points[i*4+2];
        walls[i][3] = msg->points[i*4+3];
    }
    has_map = true;

}

void get_observations(const localization::Distance_message::ConstPtr& msg){
    temp_observations[0] = msg->d1;
    temp_observations[1] = msg->d2;
    temp_observations[2] = msg->d3;
    temp_observations[3] = msg->d4;
    temp_observations[4] = msg->d5;
    temp_observations[5] = msg->d6;
    has_measurements = true;
}

void lockObserations(){
    observations_actually_observed = 0;
    for(int i=0 ;i<NUM_OBSERVATIONS;i++){
        observations[i] = temp_observations[i];
        if(observations[i]>0.01){
            observations_actually_observed+=1;
        }
    }
}

//Updates the odometry
void odom_callback(const motors::odometry::ConstPtr& msg){
    forward_movement += msg->v*msg->dt;
    rotation_movement += msg->w*msg->dt; //These are reset in the update function

    has_odom = true;
}

void calc_mean(){



    //Mean of positions
    double prob_sum = 0.0;

    double mean_x = 0.0;
    double mean_y = 0.0;

    for(int i = 0;i<NUM_PARTICLES;i++){
        mean_x += particles[i][0]*probabilities[i];
        mean_y += particles[i][1]*probabilities[i];
        prob_sum+=probabilities[i];
    }

    position.x = mean_x/prob_sum;
    position.y = mean_y/prob_sum;

    //mean of angles
    double m_th_x_vec [NUM_PARTICLES];
    double m_th_y_vec [NUM_PARTICLES];
    for(int i=0;i<NUM_PARTICLES;i++){
        m_th_x_vec[i] = cos(particles[i][2])*probabilities[i];
        m_th_y_vec[i] = sin(particles[i][2])*probabilities[i];
    }
    double m_th_x = 0.0;
    double m_th_y = 0.0;

    for(int i = 0;i<NUM_PARTICLES;i++){
        m_th_x += m_th_x_vec[i];
        m_th_y += m_th_y_vec[i];
    }
   // m_th_x = m_th_x/prob_sum;
    //m_th_y = m_th_y/prob_sum;

    position.theta = atan2(m_th_y,m_th_x);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 10;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = position.x;
    marker.pose.position.y = position.y;
    marker.pose.position.z = 0;



    marker.pose.orientation = tf::createQuaternionMsgFromYaw( position.theta );

    marker.scale.x = 0.2;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    pos_marker_pub.publish(marker);

    publishBeams(position.x, position.y, position.theta);
}

void publish_mean(){
    // position.x *= 100.0;
    // position.y *= 100.0;

    position_pub.publish(position);
}

bool setup(ros::NodeHandle nh){
    if (!nh.hasParam("has_parameters")){
        return false;
    }
    nh.getParam("/rot_spread", rot_spread);
    nh.getParam("/vel_spread", vel_spread);
    nh.getParam("/updates_before_resample", updates_before_resample);
    nh.getParam("/times_to_resample", times_to_resample);
    nh.getParam("/prob_sigma", prob_sigma);
    nh.getParam("/maze_xmax", maze_xmax);
    nh.getParam("/maze_ymax", maze_ymax);

    std::vector<double> ir_positions_temp;

    for(int i=0;i<NUM_OBSERVATIONS;i++){
        std::string temp = "/ir_positions_";
        temp += boost::lexical_cast<std::string>(i+1);
        nh.getParam(temp, ir_positions_temp);
        sensor_positions[i][0] = ir_positions_temp[0];
        sensor_positions[i][1] = ir_positions_temp[1];
        sensor_positions[i][2] = ir_positions_temp[2];
    }
    bool known_pos;
    nh.getParam("/known_pos", known_pos);
    if(known_pos){
        nh.getParam("/start_x", position.x);
        nh.getParam("/start_y", position.y);
        nh.getParam("/start_theta", position.theta);
        init_known_pos(position.x, position.y, position.theta);
    }
    else{
        //Unknown position

    }
    return true;
}



int main(int argc,char **argv){

    ros::init(argc,argv,"particle_filter");

    ros::NodeHandle n;

    if(!setup(n)){
        return 1; //Error, no parameters
    }

    odom_sub = n.subscribe("/odometry",100,odom_callback);
    obs_sub = n.subscribe("/ir_measurements", 100, get_observations);

    particles_pub = n.advertise<geometry_msgs::PoseArray>("/particles", 100);
    beams_pub = n.advertise<visualization_msgs::MarkerArray>("/beams", 100);
    ros::Publisher map_query_publisher = n.advertise<std_msgs::Bool>("/map_reader/query", 100);
    position_pub = n.advertise<localization::Position>("/position", 100);
    pos_marker_pub = n.advertise<visualization_msgs::Marker>("/pos_marker", 100);

    ros::Subscriber map_subscriber = n.subscribe("/map_reader/map", 100, read_map);


    while (!has_map && ros::ok()){
        std_msgs::Bool bool_msg;
        bool_msg.data = true;
        map_query_publisher.publish(bool_msg);
        ros::spinOnce();
    }



    std::cout << "Started particle filtering" << std::endl;

    //~~~~~THE PARTICLE FILTER ALGORITHM~~~~~\\

    int number_of_updates = 0;

    while (ros::ok()){
        //std::cout << has_odom << ' '<< has_measurements << ' ' << forward_movement << ' ' << rotation_movement << std::endl;
        if(has_measurements && has_odom && (forward_movement>0.01 || rotation_movement>0.05 || forward_movement < -0.01 || rotation_movement < -0.05)){

            std::cout << "calculating..."<<std::endl;
            lockObserations();
            update();

            get_probabilities();
            number_of_updates += 1;
            if (number_of_updates>= UPDATES_BEFORE_RESAMPLE)
            {

                //TODO: Only resample when we have a good enough probability for particles
                for(int derp = 0;derp<TIMES_TO_RESAMPLE;derp++){
                    resample();
                }
                number_of_updates = 0;
            }
            draw(); //TODO: comment or uncomment this!!!
            calc_mean();
            has_measurements = false;
            has_odom = false;

       }
        publish_mean();

        ros::spinOnce();
    }
}
