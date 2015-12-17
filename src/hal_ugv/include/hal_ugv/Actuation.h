#ifndef HAL_UGV_CONTROL_H
#define HAL_UGV_CONTROL_H

// Project includes
#include <hal_ugv/control/Emergency.h>
#include <hal_ugv/control/Hover.h>
#include <hal_ugv/control/Idle.h>
#include <hal_ugv/control/Land.h>
#include <hal_ugv/control/Takeoff.h>
#include <hal_ugv/control/AnglesHeight.h>
#include <hal_ugv/control/Velocity.h>
#include <hal_ugv/control/VelocityHeight.h>
#include <hal_ugv/control/Waypoint.h>

// ROS
#include <ros/ros.h>

namespace hal
{
  namespace ugv
  {
    // Define the gaussian types
    typedef enum 
    {
    	CONTROLLER_ANGLESHEIGHT,        
    	CONTROLLER_EMERGENCY,            
    	CONTROLLER_HOVER,                
    	CONTROLLER_IDLE,                 
      CONTROLLER_LAND,                  
      CONTROLLER_TAKEOFF,
      CONTROLLER_VELOCITYHEIGHT,
      CONTROLLER_VELOCITY,
      CONTROLLER_WAYPOINT,
      CONTROLLER_DISABLED
    } 
    ControllerType;

    // An abstract class for modelling noise
    class Actuation
    {     

    private:

      // Create services
      ros::ServiceServer srvAnglesHeight;
      ros::ServiceServer srvEmergency;
      ros::ServiceServer srvHover;
      ros::ServiceServer srvLand;
      ros::ServiceServer srvTakeoff;
      ros::ServiceServer srvVelocity;
      ros::ServiceServer srvVelocityHeight;
      ros::ServiceServer srvWaypoint;

      // One of each controller
      AnglesHeight    cAnglesHeight;
      Emergency       cEmergency;
      Hover           cHover;
      Idle            cIdle;
      Land            cLand;
      Takeoff         cTakeoff;
      Velocity        cVelocity;
      VelocityHeight  cVelocityHeight;
      Waypoint        cWaypoint;

      /// The current controller
      ControllerType  current;

    public:    

      //! Initialise the controller factory
      /*!
          \param nh the ROS node handle
          \param controller the default controller
      */
      void Init(ros::NodeHandle nh, ControllerType controller = CONTROLLER_IDLE);

      //! Switch to a new controller
      /*!
          \param controller the new controller
      */
      void Switch(ControllerType controller);

      //! Obtain control from state and timestep
      /*!
          \param state the current platform state
          \param t the current time
          \param control the output control from the controller
          \return if the state could be updated
      */
      bool GetControl(hal_ugv::State &state, double t, hal_ugv::Control &control);

      // CONTROLLER CALLBACKS //////////////////////////////////////

      //! Callback for new AnglesHeight request
      /*!
        \param req service request
        \param res service response
        \return whether the service was process successfully
      */
      bool RcvAnglesHeight(
          hal_ugv::AnglesHeight::Request  &req, 
          hal_ugv::AnglesHeight::Response &res
      );

      //! Callback for new Emergency request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvEmergency(
          hal_ugv::Emergency::Request  &req, 
          hal_ugv::Emergency::Response &res
      );
      
      //! Callback for new Hover request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvHover(
          hal_ugv::Hover::Request  &req, 
          hal_ugv::Hover::Response &res
      );
                  
      //! Callback for new Land request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvLand(
          hal_ugv::Land::Request  &req, 
          hal_ugv::Land::Response &res
      );

      //! Callback for new Takeoff request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvTakeoff(
          hal_ugv::Takeoff::Request  &req, 
          hal_ugv::Takeoff::Response &res
      );
      
      //! Callback for new Velocity request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvVelocity(
          hal_ugv::Velocity::Request  &req, 
          hal_ugv::Velocity::Response &res
      );

      //! Callback for new VelocityHeight request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvVelocityHeight(
          hal_ugv::VelocityHeight::Request  &req, 
          hal_ugv::VelocityHeight::Response &res
      );

      //! Callback for new Waypoint request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvWaypoint(
          hal_ugv::Waypoint::Request  &req, 
          hal_ugv::Waypoint::Response &res
      );

    };
  }
}

#endif
