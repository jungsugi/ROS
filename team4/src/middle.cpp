#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <vector>  
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32MultiArray.h"
#include <math.h>
#include "team4/srvChangeMode.h"
#include "team4/srvRC.h"
#include "team4/srvPaint.h"
#define PS geometry_msgs::PoseStamped
#define PUB ros::Publisher
#define SUB ros::Subscriber
#define SERVICE ros::ServiceServer
#define FL std_msgs::Float64
#define PI 3.1415926
#define MODE_RC 0
#define MODE_PAINT_STOP 1
#define MODE_PAINT_DRAW 2
#define PAINTDIV 100.0
#define PAINT_MOVING_TIME 2

double r,wn;

struct rpy{
    float roll;
    float pitch;
    float yaw;
};

//VARIABLE : ATTITUDE, POSITION
PS POSE;
struct rpy RPY;
double thr=0.0;

//VARIABLE FROM COM TO MAVROS
PS msg;
FL msg_thr;

//MODE RC, PAINT_STOP, PAINT_DRAW
int MODE=MODE_RC;

int tt=0;
int count=1;


//VARIABLE FOR PAINT
std::vector<int> pointsSet;
int pointPos=-1;
int pointSize=0;

//Initialize
void InitDroneState();

// PUB/SUB MAVROS
void movePosition(PUB& point_pub);
void moveAttitude(PUB& att_pub, PUB& thr_pub);

// Service for GUI 
bool changeModeCallback(team4::srvChangeMode::Request &req, team4::srvChangeMode::Response &res);
bool RCCallback(team4::srvRC::Request &req, team4::srvRC::Response &res);
bool PaintCallback(team4::srvPaint::Request &req, team4::srvPaint::Response &res);


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg);

int main(int argc, char **argv){    	

    InitDroneState();

	ros::init(argc, argv, "pub_team4");
	ros::NodeHandle n;

	PUB point_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",100);

	PUB att_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude",100);
	PUB thr_pub = n.advertise<std_msgs::Float64>("mavros/setpoint_attitude/att_throttle" ,100);

    SERVICE chmode_ser = n.advertiseService("srv_ChangeMode", changeModeCallback);
    SERVICE rc_ser = n.advertiseService("srv_RC",RCCallback);
    SERVICE paint_ser = n.advertiseService("srv_Paint",PaintCallback);

    ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    
    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    

	ros::Rate loop_rate(25.0);
	ros::spinOnce();

    
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;


    ros::Time last_request = ros::Time::now();

   while(ros::ok()){
/*    
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
*/

       msg.header.stamp = ros::Time::now();
       msg.header.seq=count;
       msg.header.frame_id = 1;
	
       switch(MODE){
        case MODE_RC :    moveAttitude(att_pub,thr_pub);
                          break;
        case MODE_PAINT_STOP : case MODE_PAINT_DRAW : 
                          movePosition(point_pub);
                          break;    
       }
       ros::spinOnce();
       count++;
       loop_rate.sleep();
   }
   return 0;
}

void InitDroneState(){        
    //Initialize Roll, Pitch, Yaw
    RPY.roll=0.0;
    RPY.pitch=0.0;
    RPY.yaw=0.0;

    //Initalize Position x,y,z;
    POSE.pose.position.x=0.0;
    POSE.pose.position.y=0.0;
    POSE.pose.position.z=2;
    POSE.pose.orientation.x=0.0;
    POSE.pose.orientation.y=0.0;
    POSE.pose.orientation.z=0.0;
    POSE.pose.orientation.w=1;

    thr=0.0;
}

void movePosition(PUB& point_pub){

    if(MODE==MODE_PAINT_STOP){ 
       ROS_INFO("HOVERING !!!!!-!_!-!-1-!-!-!-   %f  %f  %f",POSE.pose.position.x,POSE.pose.position.y,POSE.pose.position.z);
       msg.pose.position.x = POSE.pose.position.x;
       msg.pose.position.y = POSE.pose.position.y;
       msg.pose.position.z = POSE.pose.position.z;
       msg.pose.orientation.x = 0;
       msg.pose.orientation.y = 0;
       msg.pose.orientation.z = 0;
       msg.pose.orientation.w = 1;
    }else{
        if(pointPos<=-1 || pointPos>=pointSize) MODE=MODE_PAINT_STOP;

        msg.pose.position.x = (-pointsSet[0] + pointsSet[pointPos])/PAINTDIV;
        msg.pose.position.y = (-pointsSet[1] + pointsSet[pointPos+1])/PAINTDIV;
        msg.pose.position.z = POSE.pose.position.z;
        msg.pose.orientation.x = 0;
        msg.pose.orientation.y = 0;
        msg.pose.orientation.z = 0;
        msg.pose.orientation.w = 1;


       ROS_INFO("PAINTING!!!!!-!_!-!-1-!-!-!-   %f  %f  %f",msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);

        if(count%PAINT_MOVING_TIME==0){ 
            // Change position 100ms   
            pointPos+=2;
        }
    }   
    point_pub.publish(msg);
}



void moveAttitude(PUB& att_pub, PUB& thr_pub){

     
       ROS_INFO("RC CONTROL !!!!!-!_!-!-!-!-!-!-   %f  %f  %f %f",RPY.roll,RPY.pitch,RPY.yaw,thr);
       double c[3] = {RPY.yaw,RPY.pitch,RPY.roll};
       double s[3] = {RPY.yaw,RPY.pitch,RPY.roll};
       for(int i=0;i<3;i++){
        c[i]=cos(c[i]*PI/(2.0*180));
        s[i]=sin(s[i]*PI/(2.0*180));
       }
       // 0 yaw , 1 pitch , 2 roll; 
       msg.pose.position.x=0.0;
       msg.pose.position.y=0.0;
       msg.pose.position.z=0.0;
       
       msg.pose.orientation.w=c[0]*c[1]*c[2] + s[0]*s[1]*s[2]; 
       msg.pose.orientation.x=c[0]*c[1]*s[2] - s[0]*s[1]*c[2];
       msg.pose.orientation.y=c[0]*s[1]*c[2] + s[0]*c[1]*s[2];
       msg.pose.orientation.z=s[0]*c[0]*c[2] - c[0]*s[1]*s[2];
      

       msg_thr.data=thr;

       att_pub.publish(msg);
       thr_pub.publish(msg_thr);

}

bool changeModeCallback(team4::srvChangeMode::Request &req, team4::srvChangeMode::Response &res) {
    //Request : bool change
    //Result : bool result
    if(MODE == MODE_RC) MODE=MODE_PAINT_STOP;
    else MODE = MODE_RC;
    res.result=true;
    ROS_INFO("CHANGE MODE SERVICE !!");
    
    return true; 
}

bool RCCallback(team4::srvRC::Request &req, team4::srvRC::Response &res) {
    //Request : float64 roll
    //Request : float64 pitch
    //Request : float64 yaw
    //Request : float64 thrust
    //Result : bool result

    RPY.roll=req.roll;
    RPY.pitch=req.pitch;
    RPY.yaw=req.yaw;
    thr=req.thrust;

    res.result=true;
    ROS_INFO("CHANGE DESIRE ATTITUDE %f %f %f %f",req.roll,req.pitch,req.yaw,req.thrust);
    
    return true; 
}

bool PaintCallback(team4::srvPaint::Request &req, team4::srvPaint::Response &res) {
    //Request : int32[] points
    //        : int32 size;
    //Result : boola result
    //std_msgs::int32[] 
    pointSize = req.size;
    pointsSet= req.points;
    pointPos=0;

    for(int i=0;i<pointSize;i+=2){
        ROS_INFO("%d %d",pointsSet[i],pointsSet[i+1]);
    }
    res.result=true;
    ROS_INFO("CHANGE MODE SERVICE !!");
    if(MODE== MODE_PAINT_STOP) MODE= MODE_PAINT_DRAW;
    
    return true; 
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
