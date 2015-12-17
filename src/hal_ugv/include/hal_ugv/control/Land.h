#ifndef HAL_UGV_LAND_H
#define HAL_UGV_LAND_H

// Base controller type
#include <hal_ugv/control/Controller.h>

// Messages used by this controller
#include <hal_ugv/Land.h>

namespace hal
{
    namespace ugv
    {
        //! A ugv Emergency controller
        /*!
          A more elaborate class description.
        */
        class Land : public Controller
        {

        private:
            
            /// If we have reached the goal
            bool reach;

            // PID parameters
            double iz;
            double ez;
            double sp[4];

            // extra vars to help determine reach condition
            double t_stopped;
            bool stopped;

        public:

            //! Callback for goal update
            /*!
              \param req the goal request
              \param res the goal response
              \return whether the control was accepted
            */
            bool SetGoal(
                hal_ugv::Land::Request& req, 
                hal_ugv::Land::Response& res
            );

            //! Obtain control from state and timestep
            /*!
              \param state the current platform state
              \param dt the discrete time step
              \param control the output control from the controller
              \return if the state could be updated
            */
            void Update(const hal_ugv::State &state, 
                double dt, hal_ugv::Control &control);

            //! Goal reach implementations
            /*!
              \return Whether the goal has been reached
            */
            bool HasGoalBeenReached();

            //! Reset implementation
            /*!
              \return Reset the internal state of the controller
            */
            void Reset();
        };
    }
}

#endif
