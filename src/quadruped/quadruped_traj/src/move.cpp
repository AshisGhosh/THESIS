#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <math.h>


const float PI = 3.14159265359;
int count=0;
int phase=1;

 std::string leg;
 std::string joint;
 char command[50];
 float pos=0;

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  
  ros::NodeHandle n;

  ros::Publisher lb_ankle_pub = n.advertise<std_msgs::Float64>("quadruped/lb_ankle_position_controller/command", 1000);
  ros::Publisher lb_knee_pub = n.advertise<std_msgs::Float64>("quadruped/lb_knee_position_controller/command", 1000);
  ros::Publisher lb_shoulder_pub = n.advertise<std_msgs::Float64>("quadruped/lb_shoulder_position_controller/command", 1000);
  ros::Publisher lb_roll_shoulder_pub = n.advertise<std_msgs::Float64>("quadruped/lb_roll_shoulder_position_controller/command", 1000);

  ros::Publisher lf_ankle_pub = n.advertise<std_msgs::Float64>("quadruped/lf_ankle_position_controller/command", 1000);
  ros::Publisher lf_knee_pub = n.advertise<std_msgs::Float64>("quadruped/lf_knee_position_controller/command", 1000);
  ros::Publisher lf_shoulder_pub = n.advertise<std_msgs::Float64>("quadruped/lf_shoulder_position_controller/command", 1000);
  ros::Publisher lf_roll_shoulder_pub = n.advertise<std_msgs::Float64>("quadruped/lf_roll_shoulder_position_controller/command", 1000);

  ros::Publisher rb_ankle_pub = n.advertise<std_msgs::Float64>("quadruped/rb_ankle_position_controller/command", 1000);
  ros::Publisher rb_knee_pub = n.advertise<std_msgs::Float64>("quadruped/rb_knee_position_controller/command", 1000);
  ros::Publisher rb_shoulder_pub = n.advertise<std_msgs::Float64>("quadruped/rb_shoulder_position_controller/command", 1000);
  ros::Publisher rb_roll_shoulder_pub = n.advertise<std_msgs::Float64>("quadruped/rb_roll_shoulder_position_controller/command", 1000);

  ros::Publisher rf_ankle_pub = n.advertise<std_msgs::Float64>("quadruped/rf_ankle_position_controller/command", 1000);
  ros::Publisher rf_knee_pub = n.advertise<std_msgs::Float64>("quadruped/rf_knee_position_controller/command", 1000);
  ros::Publisher rf_shoulder_pub = n.advertise<std_msgs::Float64>("quadruped/rf_shoulder_position_controller/command", 1000);
  ros::Publisher rf_roll_shoulder_pub = n.advertise<std_msgs::Float64>("quadruped/rf_roll_shoulder_position_controller/command", 1000);
 
  std_msgs::Float64 trj_msg, lb_s_msg, lf_s_msg, rb_s_msg, rf_s_msg, lb_k_msg, lf_k_msg, rb_k_msg, rf_k_msg, lb_rs_msg, lf_rs_msg, rb_rs_msg, rf_rs_msg;

 /* if(!(count%50)){
	//count=0;
	switch(trigger1){
	case(1): trigger1=0; break;
	//case(0): direction=-1; break;
	case(0):trigger1=1; break;
	}	  
  }*/




 if(leg != "walk")
 	std::cin >> leg;
  
 /*if((leg!= "walk")&&(leg!="zero")){
	std::cin >> joint;
	std::cin >> pos;
        trj_msg.data = pos;
	
	sprintf(command,"%s_%s_pub.publish(trj_msg.data) = %f",leg.c_str(), joint.c_str(), pos);
	std::cout << command << std::endl;

	if ((leg == "lb")&&(joint=="ankle"))
		lb_ankle_pub.publish(trj_msg);
	if ((leg == "lb")&&(joint=="knee"))
		lb_knee_pub.publish(trj_msg);
	if ((leg == "lb")&&(joint=="shoulder"))
		lb_shoulder_pub.publish(trj_msg);

	if ((leg == "lf")&&(joint=="ankle"))
		lf_ankle_pub.publish(trj_msg);
	if ((leg == "lf")&&(joint=="knee"))
		lf_knee_pub.publish(trj_msg);
	if ((leg == "lf")&&(joint=="shoulder"))
		lf_shoulder_pub.publish(trj_msg);

	if ((leg == "rb")&&(joint=="ankle"))
		rb_ankle_pub.publish(trj_msg);
	if ((leg == "rb")&&(joint=="knee"))
		rb_knee_pub.publish(trj_msg);
	if ((leg == "rb")&&(joint=="shoulder"))
		rb_shoulder_pub.publish(trj_msg);

	if ((leg == "rf")&&(joint=="ankle"))
		rf_ankle_pub.publish(trj_msg);
	if ((leg == "rf")&&(joint=="knee"))
		rf_knee_pub.publish(trj_msg);
	if ((leg == "rf")&&(joint=="shoulder"))
		rf_shoulder_pub.publish(trj_msg);

	
	ros::spinOnce();
	}*/

 if(leg == "zero"){
	trj_msg.data = 0;

	lb_ankle_pub.publish(trj_msg);
	lb_knee_pub.publish(trj_msg);
	lb_shoulder_pub.publish(trj_msg);
	lb_roll_shoulder_pub.publish(trj_msg);

	lf_ankle_pub.publish(trj_msg);
	lf_knee_pub.publish(trj_msg);
	lf_shoulder_pub.publish(trj_msg);
	lf_roll_shoulder_pub.publish(trj_msg);

	rb_ankle_pub.publish(trj_msg);
	rb_knee_pub.publish(trj_msg);
	rb_shoulder_pub.publish(trj_msg);
	rb_roll_shoulder_pub.publish(trj_msg);

	rf_ankle_pub.publish(trj_msg);
	rf_knee_pub.publish(trj_msg);
	rf_shoulder_pub.publish(trj_msg);
	rf_roll_shoulder_pub.publish(trj_msg);

	ros::spinOnce();
	}

 if(leg == "ready"){
	trj_msg.data = 0;

	lb_ankle_pub.publish(trj_msg);
	lb_knee_pub.publish(trj_msg);
	lb_shoulder_pub.publish(trj_msg);
	lb_roll_shoulder_pub.publish(trj_msg);

	lf_ankle_pub.publish(trj_msg);
	lf_knee_pub.publish(trj_msg);
	lf_shoulder_pub.publish(trj_msg);
	lf_roll_shoulder_pub.publish(trj_msg);

	rb_ankle_pub.publish(trj_msg);
	rb_knee_pub.publish(trj_msg);
	rb_shoulder_pub.publish(trj_msg);
	rb_roll_shoulder_pub.publish(trj_msg);

	rf_ankle_pub.publish(trj_msg);
	rf_knee_pub.publish(trj_msg);
	rf_shoulder_pub.publish(trj_msg);
	rf_roll_shoulder_pub.publish(trj_msg);

	ros::spinOnce();
	}

 if(leg == "walk"){


	  if(count==50){
		count = 0;
		if (phase == 4)
			phase = 0;	
		phase++;
	   }
	

	  
	  /*lb_s_msg.data = -PI/24*(1 -2*(phase%2));
	  lf_s_msg.data = -PI/24*(1 -2*(phase%2));
	  rb_s_msg.data = -PI/24*(1 -2*(phase%2));
	  rf_s_msg.data = -PI/24*(1 -2*(phase%2));*/

	  lb_s_msg.data = -PI/4*(phase==4) + PI/12*(phase!=4) ;
	  lf_s_msg.data = -PI/4*(phase==1) + -PI/12*(phase!=1) ;
	  rb_s_msg.data = -PI/4*(phase==2) + PI/12*(phase!=2) ;
	  rf_s_msg.data = -PI/4*(phase==3) + -PI/12*(phase!=3) ;

	  /*lb_rs_msg.data = -PI/24*(1 -2*(phase%2));
	  lf_rs_msg.data = -PI/24*(1 -2*(phase%2));
	  rb_rs_msg.data = -PI/24*(1 -2*(phase%2));
	  rf_rs_msg.data = -PI/24*(1 -2*(phase%2));*/

	  lb_rs_msg.data = -PI/6*(phase==4);
	  lf_rs_msg.data = -PI/6*(phase==1);
	  rb_rs_msg.data = PI/6*(phase==2);
	  rf_rs_msg.data = PI/6*(phase==3);


	  /*lb_s_msg.data = -PI*0.1667*direction - 0.1;
	  lf_s_msg.data = PI*0.1667*direction - 0.1;
	  rb_s_msg.data = PI*0.1667*direction - 0.1;
	  rf_s_msg.data = -PI*0.1667*direction - 0.1;*/
	  
	  lb_shoulder_pub.publish(lb_s_msg);
	  lf_shoulder_pub.publish(lf_s_msg);
	  rb_shoulder_pub.publish(rb_s_msg);
	  rf_shoulder_pub.publish(rf_s_msg);

	  lb_roll_shoulder_pub.publish(lb_rs_msg);
	  lf_roll_shoulder_pub.publish(lf_rs_msg);
	  rb_roll_shoulder_pub.publish(rb_rs_msg);
	  rf_roll_shoulder_pub.publish(rf_rs_msg);

	  /*lb_k_msg.data = PI/6*direction - 0.1;
	  lf_k_msg.data = -PI/6*direction - 0.1;
	  rb_k_msg.data = -PI/6*direction - 0.1;
	  rf_k_msg.data = PI/6*direction - 0.1;*/

	  lb_k_msg.data = -PI/80*(phase!=4); 	// 1/10
	  lf_k_msg.data = PI/8*(phase!=1);	// 1/12
	  rb_k_msg.data = -PI/80*(phase!=2);
	  rf_k_msg.data = PI/8*(phase!=3);

	  lb_knee_pub.publish(lb_k_msg);
	  lf_knee_pub.publish(lf_k_msg);
	  rb_knee_pub.publish(rb_k_msg);
	  rf_knee_pub.publish(rf_k_msg);

	  ROS_INFO("%d",count);

	  count++;

	  trj_msg.data=-PI/12;  

	  lb_ankle_pub.publish(trj_msg);
	  //lb_knee_pub.publish(trj_msg);
	  //lb_shoulder_pub.publish(trj_msg);
	  //lb_roll_shoulder_pub.publish(trj_msg);

	  lf_ankle_pub.publish(trj_msg);
	  //lf_knee_pub.publish(trj_msg);
	  //lf_shoulder_pub.publish(trj_msg);
	  //lf_roll_shoulder_pub.publish(trj_msg);

	  rb_ankle_pub.publish(trj_msg);
	  //rb_knee_pub.publish(trj_msg);
	  //rb_shoulder_pub.publish(trj_msg);
	  //rb_roll_shoulder_pub.publish(trj_msg);

	  rf_ankle_pub.publish(trj_msg);
	  //rf_knee_pub.publish(trj_msg);
	  //rf_shoulder_pub.publish(trj_msg);
	  //rf_roll_shoulder_pub.publish(trj_msg);

	  ros::spinOnce();
  }
  
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "movement");
 
  ros::NodeHandle n;

  ros::Publisher lb_ankle_pub = n.advertise<std_msgs::Float64>("quadruped/lb_ankle_position_controller/command", 1000);
  ros::Publisher lb_knee_pub = n.advertise<std_msgs::Float64>("quadruped/lb_knee_position_controller/command", 1000);
  ros::Publisher lb_shoulder_pub = n.advertise<std_msgs::Float64>("quadruped/lb_shoulder_position_controller/command", 1000);
  ros::Publisher lb_roll_shoulder_pub = n.advertise<std_msgs::Float64>("quadruped/lb_roll_shoulder_position_controller/command", 1000);

  ros::Publisher lf_ankle_pub = n.advertise<std_msgs::Float64>("quadruped/lf_ankle_position_controller/command", 1000);
  ros::Publisher lf_knee_pub = n.advertise<std_msgs::Float64>("quadruped/lf_knee_position_controller/command", 1000);
  ros::Publisher lf_shoulder_pub = n.advertise<std_msgs::Float64>("quadruped/lf_shoulder_position_controller/command", 1000);
  ros::Publisher lf_roll_shoulder_pub = n.advertise<std_msgs::Float64>("quadruped/lf_roll_shoulder_position_controller/command", 1000);

  ros::Publisher rb_ankle_pub = n.advertise<std_msgs::Float64>("quadruped/rb_ankle_position_controller/command", 1000);
  ros::Publisher rb_knee_pub = n.advertise<std_msgs::Float64>("quadruped/rb_knee_position_controller/command", 1000);
  ros::Publisher rb_shoulder_pub = n.advertise<std_msgs::Float64>("quadruped/rb_shoulder_position_controller/command", 1000);
  ros::Publisher rb_roll_shoulder_pub = n.advertise<std_msgs::Float64>("quadruped/rb_roll_shoulder_position_controller/command", 1000);

  ros::Publisher rf_ankle_pub = n.advertise<std_msgs::Float64>("quadruped/rf_ankle_position_controller/command", 1000);
  ros::Publisher rf_knee_pub = n.advertise<std_msgs::Float64>("quadruped/rf_knee_position_controller/command", 1000);
  ros::Publisher rf_shoulder_pub = n.advertise<std_msgs::Float64>("quadruped/rf_shoulder_position_controller/command", 1000);
  ros::Publisher rf_roll_shoulder_pub = n.advertise<std_msgs::Float64>("quadruped/rf_roll_shoulder_position_controller/command", 1000);
  
  ros::Subscriber sub = n.subscribe("quadruped/joint_states", 1000, jointCallback);

  ros::spin();

  return 0;
}
