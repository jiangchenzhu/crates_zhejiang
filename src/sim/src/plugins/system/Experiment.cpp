// STL
#include <map>

// Needed for SDF parsing
#include <tinyxml2.h>

// SDF
#include <sdf/sdf.hh>

// Find HAL matches
#include <boost/regex.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// Quadrotor state
#include <hal_quadrotor/State.h>

// ROS
#include <ros/ros.h>

namespace gazebo
{
  class Experiment : public SystemPlugin
  {

  private:

    // Plugin state management
    bool loaded, created, stop;

    // Lock access to fields that are used in ROS message callbacks
    boost::mutex                            lock;

    // ROS comm
    boost::shared_ptr<ros::NodeHandle>      rosNode;
    boost::shared_ptr<ros::AsyncSpinner>    async;

    // Gazebo event handling
    event::ConnectionPtr                    eventSigint;
    event::ConnectionPtr                    eventLoad;

    // gazebo world
    physics::WorldPtr                       world;

    // For communication on gazebo backbone
    transport::NodePtr                      gazeboNode;
    transport::PublisherPtr                 pubFactory;
    transport::PublisherPtr                 pubRequest;
    transport::SubscriberPtr                subResponse;

    /// Timers
    ros::Timer                              timerSearch;
    ros::master::V_TopicInfo                topics;
    ros::V_string                           nodes;

    // Registry of all devices
    std::map<std::string,ros::Subscriber>   registry;

    // Callback for quadrotor state
    void StateCallback(std::string unit, const hal_quadrotor::State::ConstPtr& msg) 
    {
      ROS_INFO("Device %s (x,y,z) -> (%f,%f,%f)",unit.c_str(),msg->x,msg->y,msg->z);
      // Get a pointer to the model and return if does not exist
      if (world->GetModel(unit))
      {
        world->GetModel(unit)->GetLink("body")->SetWorldPose(
          math::Pose(msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw)
        );
      }
    }

  public:

    // Constructor
    Experiment() : created(false), stop(false), loaded(false)
    {
      // Do nothing
    }

    // Destructor
    ~Experiment()
    {
      // Unload the sigint event
      event::Events::DisconnectSigInt(eventSigint);
      
      // Don't attempt to unload this plugin if it was never loaded in the Load() function
      if(!loaded)
        return;
          
      // Stop the multi threaded ROS spinner
      async->stop();
    
      // Shutdown the ROS node
      rosNode->shutdown();
    }

    // Called when CTRL+C is pressed
    void Shutdown()
    {
      stop = true;
    }

    // Called to load the plugin
    void Load(int argc, char** argv)
    {
      // When a SIGINT is receiver, send the sudown signal
      eventSigint = event::Events::ConnectSigInt(
        boost::bind(&Experiment::Shutdown,this)
      );

      // Start ROS without SIGINT ability
      if (!ros::isInitialized())
        ros::init(argc,argv,"sim",ros::init_options::NoSigintHandler);
      else
        ROS_ERROR("Something started ros::init(...) prior to libbsexp Load()");

      // Wait until the ROS master becomes available
      while(!ros::master::check())
      {
        usleep(500*1000);
        if(stop)
          return;
      }

      // Advertise topics and services in this node's namespace
      rosNode.reset(new ros::NodeHandle("~")); 

      // Built-in multi-threaded ROS spinnin
      async.reset(new ros::AsyncSpinner(0)); 
      async->start();

      // On connecting to the world, initialise all services
      eventLoad = event::Events::ConnectWorldCreated(
        boost::bind(&Experiment::Init,this,_1)
      );

      // Load success
      loaded = true;
    }

    // Called when world is connected to
    void Init(std::string world_name)
    {
      // Ensures that this is called only once
      event::Events::DisconnectWorldCreated(eventLoad);
      lock.lock();
      if (created)
      {
        lock.unlock();
        return;
      }
      created = true;
      lock.unlock();

      // Check to see that the world is valid
      world = physics::get_world(world_name);
      if (!world)
      {
        ROS_FATAL("physics::get_world() fails to return a valid world pointer");
        return;
      }

      // Initialise the messaging 
      gazeboNode = transport::NodePtr(new transport::Node());
      gazeboNode->Init(world_name);

      // Set up gazebo publishers
      pubFactory  = gazeboNode->Advertise<msgs::Factory>("~/factory");
      pubRequest  = gazeboNode->Advertise<msgs::Request>("~/request");
      subResponse = gazeboNode->Subscribe("~/response",&Experiment::Response, this);

      // Advertise more services on the custom queue
      timerSearch = rosNode->createTimer(
          ros::Duration(1.0), 
          &Experiment::Search, 
          this
      );

      // Set param for use_sim_time if not set by user already
      rosNode->setParam("/use_sim_time", false);
    }

    // Subscribe to messages
    void Response(ConstResponsePtr &response)
    {
      // Do nothing
    }

    // Search for known types
    void Search(const ros::TimerEvent& event)
    {
      // Try and get a list of topics currently being braodcast
      if (ros::master::getTopics(topics))
      { 
        // Iiterate over the topics
        for (ros::master::V_TopicInfo::iterator i = topics.begin(); i != topics.end(); i++)
        {
          // Check for entities mounted on /hal/<type>/<model>/<id>
          boost::cmatch mr;
          if (boost::regex_match (i->name.c_str(), mr, boost::regex("^/hal/(.+)/(.+)/(.+)/Estimate$")))
          {
            // If we haven't yet got this device registered
            if (registry.find(mr[3].str()) == registry.end())
            {
              // Debug information
              ROS_INFO("Representing unit %s of type %s with model %s",
                mr[3].str().c_str(),mr[1].str().c_str(),mr[2].str().c_str());

              // Insert model into the world
              std::string filename = common::ModelDatabase::Instance()->GetModelFile((std::string)"model://"+mr[2].str());

              // For XML
              tinyxml2::XMLDocument   xmlDocument;
              tinyxml2::XMLPrinter    xmlPrinter;

              // Create a new tinyxml document
              if (xmlDocument.LoadFile(filename.c_str()))
              {
                ROS_WARN("Could not load model file.");
                continue;
              }

              // Get the first <sdf> element
              tinyxml2::XMLElement* xmlElementSDF = xmlDocument.FirstChildElement("sdf");
              if (!xmlElementSDF)
              {
                ROS_WARN("No <sdf> tag in model file.");
                continue;
              }

              // Get the first <model> element
              tinyxml2::XMLElement* xmlElementMODEL = xmlElementSDF->FirstChildElement("model");
              if (!xmlElementMODEL)
              {
                ROS_WARN("No <model> tag in model file.");
                continue;
              }

              // Update the model name
              xmlElementMODEL->SetAttribute("name",mr[3].str().c_str());

              // Convert from a dynamic to a static model
              xmlElementMODEL->FirstChildElement("static")->FirstChild()->SetValue("true");

              // Find and remove all plugin elements, as they are not needed in simulation
              std::vector<tinyxml2::XMLElement*> kill;
              for (tinyxml2::XMLElement* child = xmlElementMODEL->FirstChildElement("plugin"); 
                child != NULL; child = child->NextSiblingElement("plugin"))
                kill.push_back(child);
              for (int child = 0; child < kill.size(); child++)
                xmlElementMODEL->DeleteChild(kill[child]);
          
              // Extract the XML data
              xmlDocument.Print(&xmlPrinter);

              // Create and publish the message
              msgs::Factory msg;
              msg.set_sdf(xmlPrinter.CStr());
              pubFactory->Publish(msg);

              //////////////////////////////////
              // Subscribe to state callbacks //
              //////////////////////////////////

              registry[mr[3].str()] = rosNode->subscribe<hal_quadrotor::State>(
                (std::string) i->name.c_str(),
                10,
                boost::bind(&Experiment::StateCallback, this, mr[3].str(), _1)
              );

              // If everything went to plan
              ROS_INFO("Registered appearance of device %s",mr[3].str().c_str());

            }
          }
        }
      }
    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(Experiment)

}