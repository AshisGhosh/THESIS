#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <gazebo_msgs/DeleteModel.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>


#include <iostream>
#include <fstream>



const double PI = 3.14159265359;
int count=0;
int phase=1;
int countmax=35;
double stability=0;

const int arr_len = 250;
double stb_mrg_arr[arr_len]={0};
double stb_ave_old=0, stb_ave=0;
int arr_count =0;

double sh_r_offset = 0.00;
double offset_inc=0.3;
int offset_dir=1;
double stb_threshold = 0.1;

int printstart=1;

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


  if(!(count%(countmax))){
	phase++;
   }
  if (count==countmax)
	count = 0;
  if (phase == 4)
	phase = 0;

  double backstart = -3*PI/18+0.1;
  double backend = 0;
  double backstep = (backend-backstart)/2; 

  double frontstart = -PI/18;
  double frontend = PI/8;
  double frontstep = (frontend-frontstart)/2;  

  lb_s_msg.data = (backstart + (backstep)*count/countmax)*(phase==2) + (backstart+backstep + backstep*count/countmax)*(phase==3) +
			(backend -(backstep)*count/countmax)*(phase==0) + (backend-backstep - backstep*count/countmax)*(phase==1);
  lf_s_msg.data = (frontstart + (frontstep)*count/countmax)*(phase==3) + (frontstart+frontstep + frontstep*count/countmax)*(phase==0) +
			(frontend -(frontstep)*count/countmax)*(phase==1) + (frontend-frontstep - frontstep*count/countmax)*(phase==2);
  rb_s_msg.data = (backstart + (backstep)*count/countmax)*(phase==0) + (backstart+backstep + backstep*count/countmax)*(phase==1) +
			(backend -(backstep)*count/countmax)*(phase==2) + (backend-backstep - backstep*count/countmax)*(phase==3);
  rf_s_msg.data = (frontstart + (frontstep)*count/countmax)*(phase==1) + (frontstart+frontstep + frontstep*count/countmax)*(phase==2) +
			(frontend -(frontstep)*count/countmax)*(phase==3) + (frontend-frontstep - frontstep*count/countmax)*(phase==0);
   
  backstart = PI/3;
  backend = 3*PI/24;
  backstep = (backend-backstart)/2; 

  frontstart = -PI/18;
  frontend = -PI/8;
  frontstep = (frontend-frontstart)/2;


  lb_k_msg.data = (backstart + (backstep)*count/countmax)*(phase==3) + (backstart+backstep + backstep*count/countmax)*(phase==0) +
			(backend -(backstep)*count/countmax)*(phase==1) + (backend-backstep - backstep*count/countmax)*(phase==2);
  lf_k_msg.data = (frontstart + (frontstep)*count/countmax)*(phase==0) + (frontstart+frontstep + frontstep*count/countmax)*(phase==1) +
			(frontend -(frontstep)*count/countmax)*(phase==2) + (frontend-frontstep - frontstep*count/countmax)*(phase==3);
  rb_k_msg.data = (backstart + (backstep)*count/countmax)*(phase==1) + (backstart+backstep + backstep*count/countmax)*(phase==2) +
			(backend -(backstep)*count/countmax)*(phase==3) + (backend-backstep - backstep*count/countmax)*(phase==0);
  rf_k_msg.data = (frontstart + (frontstep)*count/countmax)*(phase==2) + (frontstart+frontstep + frontstep*count/countmax)*(phase==3) +
			(frontend -(frontstep)*count/countmax)*(phase==0) + (frontend-frontstep - frontstep*count/countmax)*(phase==1);


  //nominal = 0.075
  // lean left max = 0.3
	
  //float l_rs_min=0.05, r_rs_min = 0.05, l_rs_max = 0.3, r_rs_max = 0.15;

  float l_rs_min=0.075, r_rs_min = 0.075, l_rs_max = 0, r_rs_max = 0;


  lb_rs_msg.data = -l_rs_min - (l_rs_max-l_rs_min)*sh_r_offset;
  lf_rs_msg.data = -l_rs_min - (l_rs_max-l_rs_min)*sh_r_offset;
  rb_rs_msg.data = r_rs_min + (r_rs_max-r_rs_min)*sh_r_offset;
  rf_rs_msg.data = r_rs_min + (r_rs_max-r_rs_min)*sh_r_offset;


  /*lb_s_msg.data = -PI/8*(phase==4) + -PI/4*(phase!=4)*0 + -PI/12*(phase==3) ;
  lf_s_msg.data = -PI/12*(phase==1) + PI/8*(phase!=1) + PI/12*(phase==2) ;
  rb_s_msg.data = -PI/8*(phase==2) + -PI/4*(phase!=2)*0 + -PI/12*(phase==1) ;
  rf_s_msg.data = -PI/12*(phase==3) + PI/8*(phase!=3) + PI/12*(phase==4) ;*/

 /*lb_s_msg.data = -PI/4*(phase==1) + -PI/18*(phase==2) + -PI/8*(phase==3) + -PI/4*(phase==4);
 lf_s_msg.data = -PI/12*(phase==1) + 0*(phase==2) + PI/6*(phase==3)+0*(phase==4);
  rb_s_msg.data = -PI/4*(phase==3) + -PI/18*(phase==4) + -PI/8*(phase==1) + -PI/4*(phase==2);
  rf_s_msg.data = -PI/12*(phase==3) + 0*(phase==4) + PI/6*(phase==1)+0*(phase==2);*/


  /*lb_s_msg.data = -PI/6*(phase==4) + -PI/8*(phase!=4) + PI/8*(phase==2) ;
  lf_s_msg.data = -PI/8*(phase==1) + PI/8*(phase!=1) - PI/12*(phase==3) ;
  rb_s_msg.data = -PI/6*(phase==2) + -PI/8*(phase!=2) + PI/8*(phase==4) ;
  rf_s_msg.data = -PI/8*(phase==3) + PI/8*(phase!=3) + -PI/12*(phase==1) ;/*

  /*lb_rs_msg.data = -PI/24*(1 -2*(phase%2));
  lf_rs_msg.data = -PI/24*(1 -2*(phase%2));
  rb_rs_msg.data = -PI/24*(1 -2*(phase%2));
  rf_rs_msg.data = -PI/24*(1 -2*(phase%2));*/



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

  
  /*lb_k_msg.data = PI/2*(phase!=4)+ PI/8*(phase==4) + PI/6*(phase==3); 	// 1/10
  lf_k_msg.data = -PI/2*(phase!=1)+ -PI/8*(phase==1) - PI/6*(phase==2);	// 1/12
  rb_k_msg.data = PI/2*(phase!=2)+ PI/8*(phase==2) + PI/6*(phase==1) ;
  rf_k_msg.data = -PI/2*(phase!=3)+ -PI/8*(phase==3) - PI/6*(phase==2);*/

  /*lb_k_msg.data = PI/3 - PI/4*(phase==2) - PI/6*(phase==4); 	// 1/10
  lf_k_msg.data = -PI/4 + PI/4*(phase==3)+ PI/6*(phase==1);	// 1/12
  rb_k_msg.data = PI/3 - PI/4*(phase==4) - PI/6*(phase==2);
  rf_k_msg.data = -PI/4 + PI/4*(phase==1)+ PI/6*(phase==3);*/

  lb_knee_pub.publish(lb_k_msg);
  lf_knee_pub.publish(lf_k_msg);
  rb_knee_pub.publish(rb_k_msg);
  rf_knee_pub.publish(rf_k_msg);

// 	CALCULATIONS DONE

//	STAB METRICS CALCULATED


	if (arr_count++<arr_len){
		stb_mrg_arr[arr_count] = stability;		
	}

	else{
		double stb_sum =0, stb_stddev=0, temp=stb_mrg_arr[0], stb_ave_old_c = stb_ave_old;
		int sortdone=0;

		while(!sortdone){
			sortdone=1;
			for (int j=0; j<arr_len-1; j++)
				if (stb_mrg_arr[j]>stb_mrg_arr[j+1]){
					sortdone=0;
					temp = stb_mrg_arr[j+1];
					stb_mrg_arr[j+1] = stb_mrg_arr[j];
					stb_mrg_arr[j] = temp;
				}
		}
	
		/*std::ofstream arrayData("/home/ashis/Desktop/array.txt",std::ios::app);
		
		arrayData<< "***************START*************" <<std::endl;
		for(int k=0;k<arr_len;k++){
			arrayData<< stb_mrg_arr[k]<<std::endl; //Outputs array to txtFile
		 }*/

	
		for(int i=arr_len*3/4; i<arr_len; i++)
			stb_sum+=stb_mrg_arr[i];		
		stb_ave = stb_sum/(arr_len/4);
		for(int i=arr_len*3/4; i<arr_len; i++)
			stb_stddev+=(stb_ave - stb_mrg_arr[i])*(stb_ave - stb_mrg_arr[i]);
		stb_stddev/=(arr_len/4);
		
		/*offset_inc = (stb_ave-stb_ave_old_c)/stb_ave_old_c;
		if(offset_inc>stb_threshold || offset_inc<-stb_threshold)
			sh_r_offset+=offset_inc;*/



		if(stb_ave<stb_ave_old){
			if(offset_dir==1)
				offset_dir=-1;
			else if(offset_dir==-1)
				offset_dir=1;
		}	
		
		/*if(stb_ave>(stb_ave_old_c*(1+stb_threshold))||stb_ave<(stb_ave_old_c*(1-stb_threshold)))
			if(1>= sh_r_offset+(offset_inc*offset_dir) >= 0)
				sh_r_offset+=offset_inc*offset_dir;*/
		
		if(sh_r_offset+(offset_inc*offset_dir)<=1 && sh_r_offset+(offset_inc*offset_dir)>=0)
			//sh_r_offset+=offset_inc*offset_dir;		

		if(stb_ave<=(stb_ave_old_c*(1+stb_threshold))&&stb_ave>=(stb_ave_old_c*(1-stb_threshold))){	
			if(offset_inc>0.01)
				offset_inc/=2;
			else
				offset_inc*=5;
		}		


		std::ofstream stabData("/home/ashis/Desktop/stabData.txt",std::ios::app);
		printf("\n Outputted to stabData.txt ------------------------------\n");
		
		if(printstart)
			stabData << "---" << std::endl;

		printstart=0;

		stabData<< stb_ave <<", "<< stb_ave_old<<", "<< (stb_ave - stb_ave_old)/stb_ave_old<<", "<<stb_stddev<<" | "<<sh_r_offset<<", "<<offset_inc*offset_dir<<std::endl; //data to txtFile

		stb_ave_old = stb_ave;

		

		arr_count = 0;
	}

  ROS_INFO("%d | %d | %lf | %lf | %lf  | %lf",count, phase, stability, stb_ave, sh_r_offset, offset_inc*offset_dir);

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

void stabilityCallback(const std_msgs::Float64& msg)
{

	//ROS_INFO("stability: %f ",msg.data);
	stability = msg.data;
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

  ros::Subscriber sub = n.subscribe("quadruped/stability", 1000, stabilityCallback);
  ros::Subscriber sub_stability = n.subscribe("quadruped/joint_states", 1000, jointCallback);
 


  ros::spin();

  return 0;
}
