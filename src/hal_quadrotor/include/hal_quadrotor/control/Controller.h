#ifndef HAL_QUADROTOR_CONTROLLER_H
#define HAL_QUADROTOR_CONTROLLER_H

// State and control messages
#include <hal_quadrotor/State.h>
#include <hal_quadrotor/Control.h>

namespace hal
{
    namespace quadrotor
    {
        // An abstract class for modelling noise
        class Controller
        {     

        protected:

            // Time of last controller update
            double tc;

            // Clamp a value to a given range
            static double limit(const double& val, const double& minval, const double& maxval);
            
            // Rotation from navigation to body frame
            static void n2b(double rot[3], double vec[3]);
            
            // Rotation from body to navigation frame
            static void b2n(double rot[3], double vec[3]);

        public:

            // Ensures the derived class destructor is called
            virtual ~Controller() {};

            //! Obtain control from state and timestep
            /*!
              \param state the current platform state
              \param t the discrete time step
              \param control the output control from the controller
              \return if the state could be updated
            */
            virtual void Update(const hal_quadrotor::State &state, 
                double dt, hal_quadrotor::Control &control) = 0;

            //! Goal reach implementations
            /*!
              \return Whether the goal has been reached
            */
            virtual bool HasGoalBeenReached() = 0;

            //! Reset implementation
            /*!
              \return Reset the internal state of the controller
            */
            virtual void Reset() = 0;

            //! Obtain control from state and timestep
            /*!
              \param state the current platform state
              \param t the current time
              \param control the output control from the controller
              \return if the state could be updated
            */
            void GetControl(const hal_quadrotor::State &state, double t, hal_quadrotor::Control &control);

        };
    }
}

#endif