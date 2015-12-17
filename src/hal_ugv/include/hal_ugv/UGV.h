#ifndef HAL_UGV_H
#define HAL_UGV_H

// System libraries
#include <string>
#include <map>

// Basic ROS stuff
#include <hal/HAL.h>

// Core ugv functionality
#include <hal_ugv/Navigation.h>
#include <hal_ugv/Actuation.h>

// ROS Services
#include <hal_ugv/SetTruth.h>
#include <hal_ugv/SetEstimate.h>
#include <hal_ugv/SetControl.h>
#include <hal_ugv/GetTruth.h>
#include <hal_ugv/GetEstimate.h>
#include <hal_ugv/GetControl.h>

namespace hal
{
    namespace ugv
    {
        class UGV : public hal::HAL
        {       

        private:

            /// Current state of the ugv
            hal_ugv::State estimate, truth;

            /// Current control vector
            hal_ugv::Control control;

            /// Last time at which the update clock was called
            double tick;

            /// Publishers
            ros::Publisher      pubControl;          /*!< Control publisher      */
            ros::Publisher      pubEstimate;         /*!< State publisher        */
            ros::Publisher      pubTruth;            /*!< State publisher        */
            
            /// Timers
            ros::ServiceServer  srvSetTruth;         /*!< Update loop timer       */
            ros::ServiceServer  srvSetEstimate;      /*!< Update loop timer       */
            ros::ServiceServer  srvSetControl;       /*!< State broadcast timer   */
            ros::ServiceServer  srvGetTruth;         /*!< Update loop timer       */
            ros::ServiceServer  srvGetEstimate;      /*!< Update loop timer       */
            ros::ServiceServer  srvGetControl;       /*!< State broadcast timer   */

            /// Timers
            ros::Timer          timerUpdate;          /*!< Update loop timer       */
            ros::Timer          timerTruth;           /*!< State broadcast timer   */
            ros::Timer          timerEstimate;        /*!< State broadcast timer   */
            ros::Timer          timerControl;         /*!< Control broadcast timer */

            /// Converts HL-control instructions to LL-control commands
            Actuation           actuation;

            /// Converts sensor measurements to a state estimate
            Navigation          navigation;

            // CORE CONTROLLER FUNCRIONS ///////////////////////////////////////

            // Called by HAL when ROS is ready!            
            void OnInit();

            //! Timer callback for internal update loop
            /*!
              \param event Timer event
            */
            void Update(const ros::TimerEvent& event);

            // TOPIC CALLBACKS //////////////////////////////////////////////////

            //! Timer callback for state broadcast loop
            /*!
              \param event Timer event
            */
            void BroadcastTruth(const ros::TimerEvent& event);

            //! Timer callback for state broadcast loop
            /*!
              \param event Timer event
            */
            void BroadcastEstimate(const ros::TimerEvent& event);

            //! Timer callback for control broadcast loop
            /*!
              \param event Timer event
            */
            void BroadcastControl(const ros::TimerEvent& event);

            /// RECEIVE CALLS FOR ALL SENSOR DATA ////////////////////////////

            //! Service callback for getting the state
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvGetTruth(
                hal_ugv::GetTruth::Request  &req, 
                hal_ugv::GetTruth::Response &res
            );

            //! Service callback for setting the state
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvSetTruth(
                hal_ugv::SetTruth::Request  &req, 
                hal_ugv::SetTruth::Response &res
            );

            //! Service callback for getting the state
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvGetEstimate(
                hal_ugv::GetEstimate::Request  &req, 
                hal_ugv::GetEstimate::Response &res
            );

            //! Service callback for setting the state
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvSetEstimate(
                hal_ugv::SetEstimate::Request  &req, 
                hal_ugv::SetEstimate::Response &res
            );

            //! Service callback for getting the control
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvGetControl(
                hal_ugv::GetControl::Request  &req, 
                hal_ugv::GetControl::Response &res
            );

            //! Service callback for setting the control
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvSetControl(
                hal_ugv::SetControl::Request  &req, 
                hal_ugv::SetControl::Response &res
            );

        protected:

            /// RECEIVE CALLS FOR ALL SENSOR DATA ////////////////////////////
 
            //! Get a pointer to the navigation subsystem
            /*!
              \return a pointer to the navigation subsystem
            */
            Navigation* GetNavPtr();

            /// INERACTION WITH THE FLIGHT CONTROL SYSTEM ///////////////////////

            //! Get the true ugv state
            /*!
              \param state the state of the ugv
            */
            virtual void GetTruth(hal_ugv::State &state) = 0;

            //! Set the true ugv state
            /*!
              \param state the state of the ugv
            */
            virtual void SetTruth(const hal_ugv::State &state) = 0;

            //! Set the ugv control
            /*!
              \param control the control to apply
              \return time at which the control was set 
            */
            virtual double SetControl(const hal_ugv::Control &control) = 0;

        public:

            //! Create a new UGV object
            /*!
              \param node ROS node tow hich the HAL will bind
            */
            UGV();
        };
    }
}

#endif
