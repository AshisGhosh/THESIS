/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>


namespace gazebo
{

	
  class xyz
  {
   public:
      double x;   
      double y;  
      double z;   
  };

  class RobotCoG : public ModelPlugin
  {
    
    private: ros::NodeHandle rosnode;
    private: ros::Publisher stability_pub = rosnode.advertise<std_msgs::Float64>("quadruped/stability", 1000);
    
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&RobotCoG::OnUpdate, this, _1));
	
      std::string topic_name;

    

    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      
      double mass_total=0, dist =0, stb_mrg=10, cog_area=0, area=0;
      xyz mass, cog, edge1, edge2;
      
      mass.x=0;
      mass.y=0;
      mass.z=0;
      cog.x=0;
      cog.y=0;
      cog.z=0;
      edge1.x=0;
      edge1.y=0;
      edge1.z=0;
      edge2.x=0;
      edge2.y=0;
      edge2.z=0;	

      //printf("Z COG is: %f\n", this->model->GetLink("link")->GetWorldCoGPose().pos.z);
      printf("There are %lu links.\n", this->model->GetLinks().size());

      for(int it=0; it<this->model->GetLinks().size(); it++) {
    	mass_total+= this->model->GetLinks()[it]->GetInertial()->GetMass();
	mass.x+=this->model->GetLinks()[it]->GetWorldCoGPose().pos.x*this->model->GetLinks()[it]->GetInertial()->GetMass();
	mass.y+=this->model->GetLinks()[it]->GetWorldCoGPose().pos.y*this->model->GetLinks()[it]->GetInertial()->GetMass();
	mass.z+=this->model->GetLinks()[it]->GetWorldCoGPose().pos.z*this->model->GetLinks()[it]->GetInertial()->GetMass();
      }
      
      cog.x=mass.x/mass_total;
      cog.y=mass.y/mass_total;
      cog.z=mass.z/mass_total;	
  
      printf("Model: %s\n",this->model->GetName().c_str());    
      printf("Robot mass is %f\n", mass_total);
      printf("CoG is: {%f %f %f}\n", cog.x, cog.y, cog.z); 

      int contacts = this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContacts().size();
      printf("There are %lu contacts\n", this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContacts().size());
      
	for (int i=0; i<contacts-1; i++){
		//printf("%s\n", this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(i)->collision1->GetName().c_str());  
		//printf("%s\n", this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(i)->collision1->GetLink()->GetName().c_str());
		for (int j=i+1; j<contacts ; j++){
			std::string link_i, link_j;
			link_i = this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(i)->collision1->GetLink()->GetName();
			link_j = this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(j)->collision1->GetLink()->GetName();

			edge1.x = this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(i)->collision1->GetLink()->GetWorldCoGPose().pos.x;
			edge1.y = this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(i)->collision1->GetLink()->GetWorldCoGPose().pos.y;
			edge2.x = this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(j)->collision1->GetLink()->GetWorldCoGPose().pos.x;
			edge2.y = this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(j)->collision1->GetLink()->GetWorldCoGPose().pos.y;


			//printf("i: %d %s j: %d %s", i,link_i.c_str(), j, link_j.c_str());
			
			//printf("\ncog_area: %f\n", cog_area);
			
			for (int k = j+1; k<contacts; k++){
				area+= sqrt(pow(0.5*(-edge1.y*edge2.x+this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(k)->collision1->GetLink()->GetWorldCoGPose().pos.y*(-edge1.x+edge2.x)+this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(k)->collision1->GetLink()->GetWorldCoGPose().pos.x*(edge1.y-edge2.y)+edge1.x*edge2.y),2));
				//printf("area: %f\n", area);
			}

			if ((contacts>3)&&(((link_i=="rb_foot"||link_j=="rb_foot")&&(link_i=="lf_foot"||link_j=="lf_foot"))||((link_i=="rf_foot"||link_j=="rf_foot")&&(link_i=="lb_foot"||link_j=="lb_foot")))){
				continue;
			}
						
			cog_area+= sqrt(pow(0.5*(-edge1.y*edge2.x+cog.y*(-edge1.x+edge2.x)+cog.x*(edge1.y-edge2.y)+edge1.x*edge2.y),2));
			dist = sqrt(pow(((edge2.y-edge1.y)*cog.x-(edge2.x-edge1.x)*cog.y+edge2.x*edge1.y-edge2.y*edge1.x),2))/(sqrt((edge2.y-edge1.y)*(edge2.y-edge1.y)+(edge2.x-edge1.x)*(edge2.x-edge1.x)));
			
			//printf("  dist: %f\n" , dist);
			if (dist<stb_mrg){
				stb_mrg = dist;			
			}
		}
			
	}
	area*=0.5;
	cog_area*=0.98;
	
	std::string stable="STABLE";
	
	if(cog_area>area){
		stable ="UNSTABLE";
	}

	printf("%f | %f\n",cog_area,area);
	//ROS_INFO("STABLE");
	printf("%s: %f\n", stable.c_str(), cog_area/area);
	printf("Stability Margin ---  %f\n", stb_mrg);
	
      std_msgs::Float64 stb_mrg_msg;

      stb_mrg_msg.data = stb_mrg;

      stability_pub.publish(stb_mrg_msg);

      //ros::spin();
      


      this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->Clear();


	//rb_foot_collision, lf_foot_collision, lb_foot_collision, rf_foot_collision
		

      //printf("Hello World!\n");
      
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RobotCoG)
}
