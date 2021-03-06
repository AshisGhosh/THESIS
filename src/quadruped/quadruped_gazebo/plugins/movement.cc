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

#define LB_RS 0
#define LB_S 1
#define LB_K 2
#define LB_A 3
#define LF_RS 4
#define LF_S 5
#define LF_K 6
#define LF_A 7
#define RB_RS 8
#define RB_S 9
#define RB_K 10
#define RB_A 11
#define RF_RS 12
#define RF_S 13
#define RF_K 14
#define RF_A 15



const float PI = 3.14159265359;
int count=0;
int phase=1;

namespace gazebo
{

	
  class xyz
  {
   public:
      double x;   
      double y;  
      double z;   
  };

  class Movement : public ModelPlugin
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
          boost::bind(&Movement::OnUpdate, this, _1));
	
      std::string topic_name;

    

    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      
            
      printf("There are %lu joints.\n", this->model->GetJoints().size());

	for(int it=0; it<this->model->GetJoints().size(); it++) {
		printf("%s | ", this->model->GetJoints()[it]->GetName().c_str());
}
     
	
			
      if(count==75){
	count = 0;
	if (phase == 4)
		phase = 0;	
	phase++;
    }

   this->model->GetJoints()[LB_S]->SetPosition(0,-PI/6*(phase==4) + -PI/8*(phase!=4) + PI/8*(phase==2));
   this->model->GetJoints()[LF_S]->SetPosition(0,-PI/8*(phase==1) + PI/8*(phase!=1) - PI/12*(phase==3));
   this->model->GetJoints()[RB_S]->SetPosition(0,-PI/6*(phase==2) + -PI/8*(phase!=2) + PI/8*(phase==4));
   this->model->GetJoints()[RF_S]->SetPosition(0,-PI/8*(phase==3) + PI/8*(phase!=3) + -PI/12*(phase==1));
     
      
    }

     printf("\n\n %d \n\n", count++);

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Movement)
}
