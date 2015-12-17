// Standard libraries
#include <hal_ugv/UGV.h>

using namespace hal::ugv;

// Rate at which control is updated internally
#define DEFAULT_QUEUE_LENGTH     10
#define DEFAULT_UPDATE_RATE    15.0

// Called on initialization
void UGV::OnInit()
{
    // Initialise navigation
	ROS_INFO("hal_ugv OnInit");
    actuation.Init(GetRosNodePtr(), CONTROLLER_IDLE);

    // In both experiments and simulation the state and LL control can be queried
    srvGetEstimate = GetRosNodePtr().advertiseService("GetEstimate", &UGV::RcvGetEstimate, this);
    srvGetControl  = GetRosNodePtr().advertiseService("GetControl", &UGV::RcvGetControl, this);

    // If we are in a simulation, additional services are avilable
    bool isSimulated = false;
    if (GetRosNodePtr().getParam("/use_sim_time",isSimulated) && isSimulated)
    {
        // Allow the true state to be mutable (hidden state in experiments)
        srvGetTruth = GetRosNodePtr().advertiseService("GetTruth", &UGV::RcvGetTruth, this);
        srvSetTruth = GetRosNodePtr().advertiseService("SetTruth", &UGV::RcvSetTruth, this);
        
        // Allow the low-level control to be set manually (too dangerous in experiments)
        srvSetControl = GetRosNodePtr().advertiseService("SetControl", &UGV::RcvSetControl, this);
        
        // Allow the internal state exstimate to be changed (too dangerous in experiments)
        srvSetEstimate = GetRosNodePtr().advertiseService("SetEstimate", &UGV::RcvSetEstimate, this);
    }

    // Publish the estimated state. For noiseless simulations, this is equal to the Truth.
    pubTruth = hal::ugv::UGV::GetRosNodePtr().template 
        advertise<hal_ugv::State>("Truth", DEFAULT_QUEUE_LENGTH);

    // Publish the estimated state. For noiseless simulations, this is equal to the Truth.
    pubEstimate = hal::ugv::UGV::GetRosNodePtr().template 
        advertise<hal_ugv::State>("Estimate", DEFAULT_QUEUE_LENGTH);

    // Publish the control (roll, pitch, yaw, throttle) from the low-level position controller.
    pubControl = hal::ugv::UGV::GetRosNodePtr().template 
        advertise<hal_ugv::Control>("Control", DEFAULT_QUEUE_LENGTH);

    // Immediately start control loop
    timerUpdate = GetRosNodePtr().createTimer(
        ros::Duration(1.0/DEFAULT_UPDATE_RATE), 
        &UGV::Update, 
        this
    );
}

// This is called at a fixed rate of 15Hz
void UGV::Update(const ros::TimerEvent& event)
{
    // Get the truthful state from the FCS, if available (simulation only)
    GetTruth(truth);

    // If the current navigation state is valid
    if (navigation.GetState(estimate))
    {
        // Only perform an update if some time has elapsed
        if (event.current_real.toSec() - tick > 0)
        {
            // Obtain the control
            actuation.GetControl(
                estimate,                                  // current state
                event.current_real.toSec(),                // time tick
                control                                    // resultant control
            );

            // Pass the control to the FCS and set the time it was applied. The 
            // flight control system will clamp the values to an acceptable range
            // and return the exact ROS time at which the control was applied.
            control.t = SetControl(control);

            // Publish the control
            pubControl.publish(control);
        }

        // Publish state estimate
        pubEstimate.publish(estimate);
    }

    // We always know the controller with complete certainty
    truth.controller = estimate.controller;

    // Publish the truth
    pubTruth.publish(truth);

    // Save the current time tick for the next iteration
    tick = event.current_real.toSec();
}

// Constructor
UGV::UGV() : hal::HAL()
{
    // Do nothing
}

// Called when the user wants the most recent truthful state  (simulation only)
bool UGV::RcvGetTruth(
    hal_ugv::GetTruth::Request  &req, 
    hal_ugv::GetTruth::Response &res)
{
    // Get the current truthful state
    res.state = truth;

    // Success!
    return true;
}

// Called when the user wants the most recent estimated state
bool UGV::RcvGetEstimate(
    hal_ugv::GetEstimate::Request  &req, 
    hal_ugv::GetEstimate::Response &res)
{
    // Get the current estimated state
    res.state = estimate;

    // Success!
    return true;
}

// Called when the user wants the most recent control
bool UGV::RcvGetControl(
    hal_ugv::GetControl::Request  &req, 
    hal_ugv::GetControl::Response &res)
{
    // Find the control, given the current state
    res.control = control;

    // Success!
    return true;
}

// Called when the user sets the truthful state (simulation only)
bool UGV::RcvSetTruth(
    hal_ugv::SetTruth::Request  &req, 
    hal_ugv::SetTruth::Response &res)
{
    // Call the FCS to set the state
    SetTruth(req.state);

    // Notify the user
    res.success = true;
    res.status  = "Truthful state set";
    return true;
}

// Called when the user sets the state estimate (simulation only)
bool UGV::RcvSetEstimate(
    hal_ugv::SetEstimate::Request  &req, 
    hal_ugv::SetEstimate::Response &res)
{
    // Get the current estimated state
    navigation.SetState(req.state);

    // Notify the user
    res.success = true;
    res.status  = "State received. Updating simulated UAV.";
    return true;
}

// Called when the user sets the control manually (simulation only)
bool UGV::RcvSetControl(
    hal_ugv::SetControl::Request  &req, 
    hal_ugv::SetControl::Response &res)
{
    // Disable the current controller, as it will override manual control,
    // until a new controll goal is involed from service controllers/xyz
    actuation.Switch(CONTROLLER_DISABLED);

    // Save the control
    control = req.control;

    // Notify the user
    res.success = true;
    res.status  = "Control received. Disabled current position controller.";
    return true;
}

// Called when new altimeter data arrives
Navigation* UGV::GetNavPtr()
{
    return &navigation;
}
