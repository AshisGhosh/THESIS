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
#include <gazebo/physics/ode/ODELink.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <gazebo_msgs/DeleteModel.h>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

double forced=0;
namespace gazebo
{

	
  class xyz
  {
   public:
      double x;   
      double y;  
      double z;   
  };

  class surfaceremove : public ModelPlugin
  {
    
    private: ros::NodeHandle rosnode;
       
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&surfaceremove::OnUpdate, this, _1));
	
      std::string topic_name;

    

    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      
      int contacts = this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContacts().size();

      //this->model->GetLink("link")->physics::ODELink::SetLinkStatic(1);
            
      for (int i=0; i<contacts; i++){
		    	std::string model_a, model_b;
			model_a = this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(i)->collision1->GetModel()->GetName();
			model_b = this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(i)->collision2->GetModel()->GetName();
		
			

			if (((model_a == this->model->GetName())&&(model_b == "quadruped")) || ((model_a == "quadruped")&&(model_b == this->model->GetName())))
			{
			    printf("model_a: %s || model_b: %s\n", model_a.c_str(), model_b.c_str());

			    physics::ModelPtr p = this->model->GetWorld()->GetModel(this->model->GetName().c_str());
			    printf("%s in contact\n",this->model->GetName().c_str());
			    double force1=0, force2=0;
			    force1=this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(i)->wrench[i].body1Force.z;
			    force2=this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(i)->wrench[i].body2Force.z;

			    //if(model_a == "surface")
			   	 printf("force1: %f\n",force1);
			    //if(model_b == "surface")
			   	 printf("force2: %f\n",force2);
				 printf("forced: %f\n", forced);
			    
			    /*if(model_a == "surface")
			   	 printf("%s\n",this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(i)->DebugString().c_str());
			    if(model_b == "surface")
			   	 printf("%s\n",this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(i)->DebugString().c_str());*/
			    
				//this->model->GetWorld()->SetPaused(1);
				printf("\n ------------------- %s DISSAPEARS-----------from %s----------\n", this->model->GetName().c_str(), this->model->GetWorld()->GetName().c_str());
				//transport::requestNoReply(this->model->GetWorld()->GetName(), "entity_delete", this->model->GetName());
				
				ros::Publisher kill_model_pub = rosnode.advertise<std_msgs::String>("gazebo/kill_model", 1000);
				std_msgs::String kill_model;
				kill_model.data=this->model->GetName();
				kill_model_pub.publish(kill_model);
				ros::spinOnce();


				/*ros::service::waitForService("gazebo/delete_model");
				ros::ServiceClient deleteModelClient = rosnode.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
				printf("\n...deleting -1- %s ...\n", this->model->GetName().c_str());
				gazebo_msgs::DeleteModel deleteModel;
				printf("\n...deleting -2- %s ...\n", this->model->GetName().c_str());
				deleteModel.request.model_name = this->model->GetName().c_str();
				printf("\n...deleting -3- %s ...\n", this->model->GetName().c_str());				
				deleteModelClient.call(deleteModel);*/

				printf("\n...delete delivered for %s ...\n", this->model->GetName().c_str());
				//this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->Clear();		
				//this->model->GetWorld()->RemoveModel(this->model);	
			    	//p->SetWorldPose(100,0,-3,0,0,0);
		    	    	//p->Fini(); 
			    	//this->model->GetWorld()->SetPaused(0);

				

			    if((abs(force1)>=100)||(abs(force2)>=100))
			    {
				if (abs(force1)>=100)
					forced=force1;
				else
					forced=force2;
				
			    }
			} 
		}


      //this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->Clear();


	//rb_foot_collision, lf_foot_collision, lb_foot_collision, rf_foot_collision
		

      //printf("Hello World!\n");
      
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(surfaceremove)
}
