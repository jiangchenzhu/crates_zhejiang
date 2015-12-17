#ifndef HAL_UGV_VELOCITY_H
#define HAL_UGV_VELOCITY_H

// Base controller type
#include <hal_ugv/control/Controller.h>

// Messages used by this controller
#include <hal_ugv/Velocity.h>

namespace hal
{
    namespace ugv
    {
        //! A ugv Emergency controller
        /*!
          A more elaborate class description.
        */
        class Velocity : public Controller
        {

        private:
            
            // PID parameters
            double ei[3];
            double ep[3];
            double sp[4];

        public:

            //! Callback for goal update
            /*!
              \param req the goal request
              \param res the goal response
              \return whether the control was accepted
            */
            bool SetGoal(
                hal_ugv::Velocity::Request& req, 
                hal_ugv::Velocity::Response& res
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
