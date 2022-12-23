#include <ros/ros.h>
#include <gnc_functions.hpp>
#include <iq_gnc/DroneListLocal.h>
#include <iq_gnc/DroneStatusLocal.h>

std::string ros_namespace;
//коэффициент для вычисления магнитуды притягивания к цели
#define AVOID_K_GOAL 1.0 
//коэффициент для вычисления магнитуды отталкивания от объектов
#define AVOID_K_OBST 15.0
//максимальное расстояние на котором отталкивают объекты
#define OBST_MAX_DIST 5.0
//максимальное расстояние на котором меняется скорость
#define OBST_CHANGE_SPEED_MAX_DIST 25.0
#define MAX_SPEED 5.0
#define MIN_SPEED 1.0
//коэффициент в формуле вычисления изменения скорости
const float CHANGE_SPEED_K = 2.0*(MAX_SPEED-MIN_SPEED)/pow(1.0/OBST_CHANGE_SPEED_MAX_DIST - 1.0/OBST_MAX_DIST,2);
iq_gnc::DroneStatusLocal current_communication_self_local;
void callback_self(const iq_gnc::DroneStatusLocal::ConstPtr& msg){
	current_communication_self_local = *msg;
}
void callback(const iq_gnc::DroneListLocal::ConstPtr& msg)
{
	iq_gnc::DroneListLocal data = *msg;
	float avoidance_vector_x = 0;
	float avoidance_vector_y = 0;
	float U = 0;
	float min_dist = OBST_CHANGE_SPEED_MAX_DIST;
	for(iq_gnc::DroneStatusLocal drone: data.drones){
		float x = -(drone.position.x - current_communication_self_local.position.x);
		float y = -(drone.position.y - current_communication_self_local.position.y);
		float dist = sqrt(x*x+y*y);
		if(dist < min_dist)
			min_dist = dist;
		if(dist < OBST_MAX_DIST)
			U = 0.5*AVOID_K_OBST*pow(1.0/dist-1.0/OBST_MAX_DIST,2);
		else
			U=0;
		avoidance_vector_x += U*x;
		avoidance_vector_y += U*y;
	}
	
	float speed = 0.5*CHANGE_SPEED_K*pow(1.0/min_dist-1.0/OBST_MAX_DIST,2) + MIN_SPEED;
	if(speed>MAX_SPEED)
		speed = MAX_SPEED;
	set_speed(speed);
	float dist = sqrt(avoidance_vector_x*avoidance_vector_x + avoidance_vector_y*avoidance_vector_y);
	if(dist > OBST_MAX_DIST){
		avoidance_vector_x = avoidance_vector_x/dist*OBST_MAX_DIST;
		avoidance_vector_y = avoidance_vector_y/dist*OBST_MAX_DIST;
	}
	ROS_INFO("v%d (%f; %f) d:%fs:%f",current_communication_self_local.sysid, avoidance_vector_x, avoidance_vector_y, min_dist, speed);
	if (dist>0.01){
		waypoint_g.pose.position.x = current_pose_g.pose.pose.position.x + avoidance_vector_x;
		waypoint_g.pose.position.y = current_pose_g.pose.pose.position.y + avoidance_vector_y;
		
		// waypoint_g.pose.position.z = current_communication_self_local.position.z;
		local_pos_pub.publish(waypoint_g);
	}
}

int main(int argc, char **argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle n("~");
	
	if (!n.hasParam("namespace"))
	{
		ROS_INFO("using default namespace");
	}else{
		n.getParam("namespace", ros_namespace);
		ROS_INFO("using namespace %s", ros_namespace.c_str());
	}
	//ros::Subscriber sub = n.subscribe("/c", 1, scan_cb);
	//initialize control publisher/subscribers
	init_publisher_subscriber(n);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	//wait4start();

	//create local reference frame 
	//initialize_local_frame();
	waypoint_g.pose.position.z = 2;
	ros::Subscriber avoidance_sub = n.subscribe<iq_gnc::DroneListLocal>((ros_namespace+"/communication/local").c_str(), 1, callback);
	ros::Subscriber avoidance_self = n.subscribe<iq_gnc::DroneStatusLocal>((ros_namespace+"/communication/self/local").c_str(), 1, callback_self);
	set_speed(MAX_SPEED);
	//request takeoff
	//takeoff(2);


	//set_destination(0,0,2,0);

	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(10.0);
	int counter = 0;
	while(ros::ok())
	{
		
		ros::spinOnce();
		rate.sleep();
		
	
	
	}

	return 0;
}

