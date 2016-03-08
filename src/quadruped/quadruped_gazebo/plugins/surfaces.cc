#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/DeleteModel.h>
#include <string>

namespace gazebo
{
class surfaces : public WorldPlugin
{

  private: ros::NodeHandle rosnode;
  physics::WorldPtr world_;
  //boost::shared_ptr<boost::thread> removeThead_;
  
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
   
   this->world_ = _parent;
   this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&surfaces::OnUpdate, this, _1));
    

	sdf::SDF surfaceSDF;
    surfaceSDF.SetFromString(
       "<sdf version ='1.4'>\
          <model name ='surface'>\
            <pose>0 0 0 0 0 0</pose>\
             <link name ='link'>\
              <pose>0 0 0 0 0 0</pose>\
              <collision name ='collision'>\
                <geometry>\
                  <box><size>15.0 2.0 0.1</size></box>\
                </geometry>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                  <box><size>15.0 2.0 0.1</size></box>\
                </geometry>\
		<material>\
          <script>\
            <uri>file://media/materials/scripts/gazebo.material</uri>\
            <name>Gazebo/Blue</name>\
          </script>\
        </material>\
              </visual>\
            </link>\
	<link name='world'/>\
	     <joint name='worldframe' type='fixed'>\
	    <parent>world</parent>\
	    <child>link</child>\
	    <pose>0 0 -3 0 0 0</pose>\
	  </joint>\
          </model>\
        </sdf>");
    // Demonstrate using a custom model name.
    //sdf::ElementPtr model = surfaceSDF.root->GetElement("model");
    //model->GetAttribute("name")->SetFromString("surface");
   // _parent->InsertModelSDF(surfaceSDF);
 

        //removeThead_ = boost::shared_ptr<boost::thread>(new boost::thread(&surfaces::Remove, this));
	//Remove();

/*printf("do we even come here?"); 
    for(int j=0;j<50;j++){
		int contacts = _parent->GetPhysicsEngine()->GetContactManager()->GetContacts().size();
		   
	    	for (int i=0; i<contacts; i++){
		    	std::string model_a, model_b;
			model_a = world_->GetPhysicsEngine()->GetContactManager()->GetContact(i)->collision1->GetModel()->GetName();
			model_b = world_->GetPhysicsEngine()->GetContactManager()->GetContact(i)->collision2->GetModel()->GetName();
		
			printf("model_a: %s || model_b: %s", model_a.c_str(), model_b.c_str());

			if (model_a == "surface" || model_b == "surface")
			{
			    sleep(5);
			    physics::ModelPtr p = world_->GetModel("surface");
			    printf("%s in contact\n","surface");
		    	    p->Fini();  
			} 
		}

		if (j>=48)
		printf("we're at the end!\n");	
    }*/
	
	
    
  }

   public: void OnUpdate(const common::UpdateInfo & /*_info*/){
      

    
   	//while(1){
				int mod_num;
				int contacts = world_->GetPhysicsEngine()->GetContactManager()->GetContacts().size();
		printf("do we even come here?");    

	    	for (int i=0; i<contacts; i++){
		    	std::string model_a, model_b;
			model_a = world_->GetPhysicsEngine()->GetContactManager()->GetContact(i)->collision1->GetModel()->GetName();
			model_b = world_->GetPhysicsEngine()->GetContactManager()->GetContact(i)->collision2->GetModel()->GetName();
			
			if (model_a != "quadruped"&& model_a!="ground_plane"){
				//mod_num = atoi(model_a.back());
				}
			
			   
			
			
			if (model_a != "ground_plane" && model_b != "ground_plane")
				{
					double force1=0, force2=0, forced=0;
				    printf("\nmodel_a: %s || model_b: %s", model_a.c_str(), model_b.c_str());
				    physics::ModelPtr p;
				    if (model_a == "quadruped")
				   	p = world_->GetModel(model_b);
				    else
					p = world_->GetModel(model_a);
				    
				    printf("\n%s in contact\n",p->GetName().c_str());
			    	    
				force1=p->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(i)->wrench[i].body1Force.z;
			    	force2=p->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(i)->wrench[i].body2Force.z;
				

				if((abs(force1)>=50000)||(abs(force2)>=50000))
				    {
					if (abs(force1)>=100)
						forced=force1;
					else
						forced=force2;
				
				    
					world_->SetPaused(1);
					//world_->GetPhysicsEngine()->GetContactManager()->Clear();
					printf("\n...deleting -3- %s ...\n", p->GetName().c_str());
					printf("\n...unfreezing %s it's %d and %d...\n", p->GetLinks()[0]->GetName().c_str(), p->IsStatic(),p->GetLinks()[0]->IsStatic());
					//world_->GetPhysicsEngine()->GetContactManager()->GetContact(i)->collision1->Fini();
					p->SetStatic(0);
					//transport::requestNoReply(world_->GetName(), "entity_delete", p->GetName());
				
					//p->Fini(); 
					p->SetWorldPose(math::Pose(100,0,-3,0,0,0));		
					//p->SetLinearVel(math::Vector3(0,0,5));
					//printf("\n...unfreezing %s it's %d and %d...\n", p->GetLinks()[0]->GetName().c_str(), p->IsStatic(),p->GetLinks()[0]->IsStatic());				
						
					world_->SetPaused(0);
					//break;
					//transport::requestNoReply(world_->GetName(), "entity_delete", p->GetName());				
				 
				     }
				
				} 
		}

	    //}
	}

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
};	

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(surfaces)
}
