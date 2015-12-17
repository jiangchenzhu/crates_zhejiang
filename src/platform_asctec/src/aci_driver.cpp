/* 
    This class provides a basic interface to an Ascending Technologies flight control system.
    The FCS provides the ability to request variables, send commands and set parameters. The
    stock C ACI library will be used from Ascec, which enforces some limitations on the code.
    In particular, one cannot pass a non-static class method as a callback to a C library,
    which means that all callbacks (denoted cb_XXX) are in the ::platform_complacs namespace. 
    This would be a problem for multiple class instantiations, but since there should be a 1
    to 1 relationship between the FCS and ROS node, this should not cause any problems.

    DATA INFORMATION

    1. RO_ALLDATA is filled at 1000Hz, but with variables at the following frequency...
        a. Max 1000Hz (angles and angular velocities) or 
        b. Max  333Hz (accelerations, magnetic measurements, height and climb/sink rate).
        c. Max    5Hz (gps)
    2. The maximum number of variable packets is 3 (we have chosen a, b, c above). Thi can be
       increased, but requires a recompilation of the on-board firmware.
    2. The maximum number of command packets is 3. We will use one for settings, another for
       arming the motors and the last for sending control commands
    3. We are using the platform in 'attitude and thrust control mode' : commands are 
       interpreted as remote control stick inputs and therefore used as inputs for standard 
       attitude controller: desired pitch, desired roll, desired yaw rate and thrust.

    USING AN ASCTEC QUAD

    1. Turn on the UAV with the remote control, and move the throttle to its lowest point.
    2. Wait for sufficient satellites.
    2. Turn the serial switch to enabled.
    3. Start the control .

*/

#define DEBUG false

// System libraries
#include <boost/utility.hpp>

// Integration with the CRATES HAL
#include <hal_quadrotor/Quadrotor.h>
#include <hal_sensor_altimeter/Altimeter.h>
#include <hal_sensor_compass/Compass.h>
#include <hal_sensor_gnss/GNSS.h>
#include <hal_sensor_imu/IMU.h>
#include <hal_sensor_orientation/Orientation.h>

// Required serial engine
#include "serial/AsyncSerial.h"

// Required ACI enfine
extern "C"
{
    #include "aci/asctecCommIntf.h"
}

// Basic math
#define MATH_PI 3.14159265359

// Whether we should use fused estimates (will limit maximum tilt angle)
#define USE_FUSED                   false

// Possible UAV states
#define STATE_ATTITUDE_CONTROL      0x0001 // ATTITUDE CONTROL 
#define STATE_STATUS_HEIGHTCTL      0x0002 // HEIGHT CONTROL
#define STATE_POSITION_CONTROL      0x0004 // POSITION CONTROL
#define STATE_COMPASS_FAILURE       0x0010 // COMPASS FAILURE
#define STATE_SERIAL_ENABLED        0x0020 // SERIAL INTERFACE ENABLED
#define STATE_SERIAL_ACTIVE         0x0040 // SERIAL INTERFACE ACTIVE - is active when attitude commands are sent to the LL
#define STATE_EMERGENCY_MODE        0x0080 // EMERGENCY MODE - when RC link is lost -> serial interface disabled
#define STATE_CALIBRATION_ERROR     0x0100 // CALIBRATION ERROR
#define STATE_GYR_CAL_ERROR         0x0200 // GYRO CALIBRATION ERRO
#define STATE_ACC_CAL_ERROR         0x0400 // ACC CALIBRATION ERROR
#define STATE_MAG_STRENGTH_ERROR    0x4000 // MAGNETIC FIELD STRENGTH ERROR
#define STATE_MAG_INCLINATION_ERROR 0x8000 // MAGNETIC INCLINATION ERROR 


// Variables
#define VAR_STATUS                  0x0001  // [ INT16] UAV status information (see above)
#define VAR_FLIGHT_TIME             0x0002  // [ INT16] Total flight time  (s)
#define VAR_BATTERY_VOLTAGE         0x0003  // [ INT16] Battery voltage (mV)
#define VAR_HL_CPU_LOAD             0x0004  // [ INT16] High-level CPU load (Hz)
#define VAR_HL_UPTIME               0x0005  // [ INT16] High-level up-time  (ms)
#define VAR_RRM_MOTOR0              0x0100  // [ UINT8] Quadcopter: front, Hexcopter front-left -- RPM measurements (0..200)
#define VAR_RRM_MOTOR1              0x0101  // [ UINT8] Quadcopter: rear, Hexcopter left -- RPM measurements (0..200)
#define VAR_RRM_MOTOR2              0x0102  // [ UINT8] Quadcopter: left, Hexcopter rear-left -- RPM measurements (0..200)
#define VAR_RRM_MOTOR3              0x0103  // [ UINT8] Quadcopter: right, Hexcopter rear-right -- RPM measurements (0..200)
#define VAR_RRM_MOTOR4              0x0104  // [ UINT8] Quadcopter: N/A, Hexcopter right -- RPM measurements (0..200)
#define VAR_RRM_MOTOR5              0x0105  // [ UINT8] Quadcopter: N/A, Hexcopter front-right -- RPM measurements (0..200)
#define VAR_GPS_LAT                 0x0106  // [ INT32] Latitude from the GPS sensor (degrees * 10^7)
#define VAR_GPS_LON                 0x0107  // [ INT32] Longitude from the GPS sensor (degrees * 10^7)
#define VAR_GPS_HEIGHT              0x0108  // [ INT32] Height from the GPS sensor (mm)
#define VAR_GPS_VEL_EW              0x0109  // [ INT32] Speed in East/West from the GPS sensor (mm/s)
#define VAR_GPS_VEL_NS              0x010A  // [ INT32] Speed in North/South from the GPS sensor (mm/s)
#define VAR_GPS_HEADING             0x010B  // [ INT32] GPS Heading  (deg * 1000)
#define VAR_GPS_POS_ACC             0x010C  // [UINT32] GPS position accuracy estimate  (mm)
#define VAR_GPS_HEIGHT_ACC          0x010D  // [UINT32] GPS height accuracy estimate (mm)
#define VAR_GPS_VEL_ACC             0x010E  // [UINT32] GPS speed accuracy estimate (mm/s)
#define VAR_GPS_NUM_SATS            0x010F  // [UINT32] Number of satellites used in NAV solution (count)
#define VAR_GPS_STATUS              0x0110  // [ INT32] GPS status information  B[7:3]: 0, B[2]: long dir; B[1]: lat dir, B[0]: GPS lock
#define VAR_GPS_TIME_OF_WEEK        0x0111  // [UINT32] Time of the week, 1 week = 604,800 s (ms)
#define VAR_GPS_WEEK                0x0112  // [ INT32] Week counter since 1980 (count)
#define VAR_GYR_CAL_Y               0x0200  // [ INT32] Pitch angle velocity  (0.0154 degree/s, bias free)
#define VAR_GYR_CAL_X               0x0201  // [ INT32] Roll angle velocity (0.0154 degree/s, bias free)
#define VAR_GYR_CAL_Z               0x0202  // [ INT32] Yaw angle velocity  (0.0154 degree/s, bias free)
#define VAR_ACC_CAL_X               0x0203  // [ INT16] Acc-sensor output in x, body frame coordinate system (calibrated: -10000..+10000 = -1g..+1g)
#define VAR_ACC_CAL_Y               0x0204  // [ INT16] Acc-sensor output in y, body frame coordinate system (calibrated: -10000..+10000 = -1g..+1g)
#define VAR_ACC_CAL_Z               0x0205  // [ INT16] Acc-sensor output in z, body frame coordinate system (calibrated: -10000..+10000 = -1g..+1g)
#define VAR_MAG_CAL_X               0x0206  // [ INT32] Magnetic field sensors output in x (Offset free and scaled to +-2500 = +- earth field strength)
#define VAR_MAG_CAL_Y               0x0207  // [ INT32] Magnetic field sensors output in y (Offset free and scaled to +-2500 = +- earth field strength)
#define VAR_MAG_CAL_Z               0x0208  // [ INT32] Magnetic field sensors output in z (Offset free and scaled to +-2500 = +- earth field strength)
#define VAR_EST_ANG_Y               0x0300  // [ INT32] Pitch angle derived by integration of gyro outputs and drift compensated by data fusion (degree*1000, Range: -90000..+90000)
#define VAR_EST_ANG_X               0x0301  // [ INT32] Roll angle derived by integration of gyro outputs and drift compensated by data fusion (degree*1000, Range: -90000..+90000)
#define VAR_EST_ANG_Z               0x0302  // [ INT32] Yaw angle derived by integration of gyro outputs and drift compensated by data fusion (degree*1000, Range: -90000..+90000)
#define VAR_EST_POS_X               0x0303  // [ INT32] Fused latitude with all other sensors (best estimations) (degrees * 10^7)
#define VAR_EST_POS_Y               0x0304  // [ INT32] Fused longitude with all other sensors (best estimations) (degrees * 10^7)
#define VAR_EST_POS_Z               0x0306  // [ INT32] Height after data fusion (mm)
#define VAR_EST_VEL_Z               0x0305  // [ INT32] Differential height after data fusion (mm/s)
#define VAR_EST_VEL_X               0x0307  // [ INT16] Fused speed in East/West with all other sensors (best estimations)  INT16   mm/s    since V1.0
#define VAR_EST_VEL_Y               0x0308  // [ INT16] Fused speed in North/South with all other sensors (best estimations)    INT16   mm/s    since V1.0
#define VAR_CHANNEL0                0x0600  // [UINT16] channel[0] Pitch command received from the remote control (0..4095)
#define VAR_CHANNEL1                0x0601  // [UINT16] channel[1] Roll command received from the remote control (0..4095)
#define VAR_CHANNEL2                0x0602  // [UINT16] channel[2] Thrust command received from the remote control (0..4095)
#define VAR_CHANNEL3                0x0603  // [UINT16] channel[3] Yaw command received from the remote control (0..4095)
#define VAR_CHANNEL4                0x0604  // [UINT16] channel[4] Serial interface enable/disable (>2048 enabled, else disabled)
#define VAR_CHANNEL5                0x0605  // [UINT16] channel[5] Manual / height control / GPS + height control  (0 -> manual mode; 2048 -> height mode; 4095 -> GPS mode)
#define VAR_CHANNEL6                0x0606  // [UINT16] channel[6] Custom remote control data (n/a)
#define VAR_CHANNEL7                0x0607  // [UINT16] channel[7] Custom remote control data (n/a)

// Commands
#define CMD_DIMC_MOTOR_0            0x0500  // [ UINT8] Direct individual motor control, Quadcopter: front, Hexcopter: front-left (0..200 = 0..100 %; 0 = motor off!)
#define CMD_DIMC_MOTOR_1            0x0501  // [ UINT8] Direct individual motor control, Quadcopter: rear, Hexcopter: left (0..200 = 0..100 %; 0 = motor off!)
#define CMD_DIMC_MOTOR_2            0x0502  // [ UINT8] Direct individual motor control, Quadcopter: left, Hexcopter: rear-left (0..200 = 0..100 %; 0 = motor off!)
#define CMD_DIMC_MOTOR_3            0x0503  // [ UINT8] Direct individual motor control, Quadcopter: right, Hexcopter: rear-right (0..200 = 0..100 %; 0 = motor off!)
#define CMD_DIMC_MOTOR_4            0x0504  // [ UINT8] Direct individual motor control, Quadcopter: N/A, Hexcopter: right (0..200 = 0..100 %; 0 = motor off! only used by the AscTec Firefly)
#define CMD_DIMC_MOTOR_5            0x0505  // [ UINT8] Direct individual motor control, Quadcopter: N/A, Hexcopter: front-right (0..200 = 0..100 %; 0 = motor off!  only used by the AscTec Firefly)
#define CMD_DMC_PITCH               0x0506  // [ UINT8] Direct motor control, Pitch rate (0..200 = - 100..+100%)
#define CMD_DMC_ROLL                0x0507  // [ UINT8] Direct motor control, Roll rate (0..200 = - 100..+100%)
#define CMD_DMC_YAW                 0x0508  // [ UINT8] Direct motor control, Yaw rate (0..200 = - 100..+100%)
#define CMD_DMC_THRUST              0x0509  // [ UINT8] Direct motor control, Thrust (0..200 = 0..100%)
#define CMD_ATT_PITCH               0x050A  // [ INT16] Attitude control, Pitch angle (-2047..+2047 (0=neutral))
#define CMD_ATT_ROLL                0x050B  // [ INT16] Attitude control, Roll angle (-2047..+2047 (0=neutral))
#define CMD_ATT_YAW                 0x050C  // [ INT16] Attitude control, Yaw angle (-2047..+2047 (0=neutral))
#define CMD_ATT_THROTTLE            0x050D  // [ INT16] Attitude control, Throttle (0..4095 = 0..100%)
#define CMD_ATT_MASK                0x050E  // [ INT16] Control byte for attitude control (bit 0: pitch, bit 1: roll, bit 2: yaw, bit 3: thrust, bit 4: height, bit 5: GPS position)
#define CMD_CTRL_MODE               0x0600  // [ UINT8] Parameter to set control mode (0x00: direct individual motor control (DIMC), 0x01: direct motor control using standard output mapping (DMC), 0x02: attitude and throttle control (CTRL), 0x03: GPS waypoint control)
#define CMD_CTRL_ENABLED            0x0601  // [ UINT8] Control commands are accepted/ignored by LL processor   UINT8   0x00: ignored, 0x01: accepted)
#define CMD_CTRL_STICKCTL           0x0602  // [ UINT8] Setting if motors can be turned on by using the stick input UINT8   0x00: disable (motors can not be turned on/off by "minimum thrust + full yaw" command), 0x01 enable (motors can be turned on/off by "minimum thrust + full yaw" command)

// Parameters
#define PAR_BATTERY_V_WARN_HIGH     0x0001  // [UINT16] Upper battery warning level -- battery almost empty (mV)
#define PAR_BATTERY_V_WARN_LOW      0x0002  // [UINT16] Lower battery warning level -- battery empty  (mV)
#define PAR_ACOUSTIC_WARNINGS       0x0003  // [ UINT8] Enable/Disable acoustic warnings  (system init 0x01, GPS quality warning 0x02)
#define PAR_PTU_CAM_OPTION4         0x0004  // [ UINT8] Version of Pelican/Firefly PanTilt camera mount option 4 (1 or 2)
#define PAR_CAM_ANGLE_ROLL_OFF      0x0400  // [ INT32] Camera roll angle offset  (0.001deg)
#define PAR_CAM_ANGLE_PITCH_OFF     0x0401  // [ INT32] Camera pitch angle offset (0.001deg)

// LOOP RATES AND FLAGS ///////////////////////////////////////////////////////////

// Receive flags
bool rcv_aci = false;
bool rcv_var = false;
bool rcv_cmd = false;
bool ready   = false;
bool error   = false;

// Data polling rates
int rate_imu = 15;      // Rate of state estimate
int rate_sen = 15;      // Rate of state estimate
int rate_pos = 15;      // Position GNSS

// Origin position (LTP <-> WGS84 conversion)
std::string port = "/dev/ttyUSB0";
int baud = 57600;
double latitude  = 51.710979902;
double longitude = -0.210839049;
double altitude  = 141.1027;

// SERIAL CALLBACKS ///////////////////////////////////////////////////////////////

// Handle to the serial port
CallbackAsyncSerial serial;

// Asycronous callback to transmit data to the serial port
void cb_tx(void* byte, unsigned short cnt)
{
    if (serial.isOpen())
        serial.write((const char*)byte,(size_t)cnt);
}

// Asynchronous callback to receive data from the serial port
void cb_rx(const char* byte, size_t cnt)
{
    for (size_t i = 0; i < cnt; i++)
        aciReceiveHandler(byte[i]);
}

// Heartbeat callback for the ACI engine
void cb_hb(const ros::TimerEvent& event)
{
    // This needs to be called to keep the ACI interface alive
    aciEngine();

    // Heartbeat received
    rcv_aci = true;
}

// COMMANDS //////////////////////////////////////////////////////////////////////

// Raw control messages
struct _raw_ctl
{
    int16_t p;
    int16_t r;
    int16_t y;
    int16_t t;
    int16_t m;
    unsigned char mode;
    unsigned char enabled;
    unsigned char stick;
} raw_ctl;

// This is called when the ACI engine's internal list of commands is updated
// At this point we are able to setup our own custom packets
void cb_cmdlist(void)
{
    ROS_INFO("Command list received from ACI device");

    // This is the packet that will be used to send motor control commands
    ROS_INFO("Creating control packet");
    aciAddContentToCmdPacket(0, CMD_ATT_PITCH,      &raw_ctl.p);
    aciAddContentToCmdPacket(0, CMD_ATT_ROLL,       &raw_ctl.r);
    aciAddContentToCmdPacket(0, CMD_ATT_YAW,        &raw_ctl.y);
    aciAddContentToCmdPacket(0, CMD_ATT_THROTTLE,   &raw_ctl.t);

    aciAddContentToCmdPacket(1, CMD_ATT_MASK,       &raw_ctl.m);
    aciAddContentToCmdPacket(1, CMD_CTRL_MODE,      &raw_ctl.mode);
    aciAddContentToCmdPacket(1, CMD_CTRL_ENABLED,   &raw_ctl.enabled);
    aciAddContentToCmdPacket(1, CMD_CTRL_STICKCTL,  &raw_ctl.stick);

    ROS_INFO("Updating command configuration");
    aciSendCommandPacketConfiguration(0,0);
    aciSendCommandPacketConfiguration(1,1);

    ROS_INFO("Sending initial DIMC commands");
#if USE_FUSED
    raw_ctl.m           = 59;   // Pitch, roll, yaw, throttle + height ctl + gps_ctl
#else
    raw_ctl.m           = 27;   // Pitch, roll, yaw, throttle + height ctl
#endif
    raw_ctl.mode        = 2;
    raw_ctl.enabled     = 1;
    raw_ctl.stick       = 1;
    aciUpdateCmdPacket(1);

    // Commands received
    rcv_cmd = true;
}

// VARIABLES ////////////////////////////////////////////////////////////////////

// Raw IMU messages
static struct _raw_imu
{
    int32_t     gyr_x;
    int32_t     gyr_y;
    int32_t     gyr_z;
    int16_t     acc_x;
    int16_t     acc_y;
    int16_t     acc_z;
} raw_imu;

// Raw state messages
static struct _raw_pos
{
    int32_t     pos_x;
    int32_t     pos_y;
    int32_t     pos_z;
    int32_t     ang_x;
    int32_t     ang_y;
    int32_t     ang_z;
    int16_t     vel_x;
    int16_t     vel_y;
    int32_t     vel_z;
    int32_t     mag_x;
    int32_t     mag_y;
    int32_t     mag_z;
    int32_t     latitude;
    int32_t     longitude;
    int32_t     altitude;
    int32_t     vel_ew;
    int32_t     vel_ns;
    int32_t     heading;
    uint32_t    pdop;
    uint32_t    hdop;
    uint32_t    vdop;
    uint32_t    numsvs;
} raw_pos;

// This is called when the ACI engine's internal list of variables is updated
// At this point we are able to setup our own custom packets
void cb_varlist(void)
{
    ROS_INFO("Variable list received from ACI device");

    int pkt = 1;

    if (rate_imu > 0)
    {
        ROS_INFO("Setting up IMU data in packet %d",pkt);
        aciAddContentToVarPacket(pkt,VAR_GYR_CAL_X,           &raw_imu.gyr_x);
        aciAddContentToVarPacket(pkt,VAR_GYR_CAL_Y,           &raw_imu.gyr_y);
        aciAddContentToVarPacket(pkt,VAR_GYR_CAL_Z,           &raw_imu.gyr_z);
        aciAddContentToVarPacket(pkt,VAR_ACC_CAL_X,           &raw_imu.acc_x);
        aciAddContentToVarPacket(pkt,VAR_ACC_CAL_Y,           &raw_imu.acc_y);
        aciAddContentToVarPacket(pkt,VAR_ACC_CAL_Z,           &raw_imu.acc_z);
        aciSetVarPacketTransmissionRate(pkt,10);
        pkt++;
    }

    if (rate_pos > 0)
    {
        ROS_INFO("Setting up POS data in packet %d",pkt);        
        aciAddContentToVarPacket(pkt,VAR_GPS_LAT,             &raw_pos.latitude);
        aciAddContentToVarPacket(pkt,VAR_GPS_LON,             &raw_pos.longitude);
        aciAddContentToVarPacket(pkt,VAR_GPS_HEIGHT,          &raw_pos.altitude);
        aciAddContentToVarPacket(pkt,VAR_GPS_VEL_NS,          &raw_pos.vel_ns);
        aciAddContentToVarPacket(pkt,VAR_GPS_VEL_EW,          &raw_pos.vel_ew);
        aciAddContentToVarPacket(pkt,VAR_GPS_HEADING,         &raw_pos.heading);
        aciAddContentToVarPacket(pkt,VAR_GPS_POS_ACC,         &raw_pos.pdop);
        aciAddContentToVarPacket(pkt,VAR_GPS_HEIGHT_ACC,      &raw_pos.hdop);
        aciAddContentToVarPacket(pkt,VAR_GPS_VEL_ACC,         &raw_pos.vdop);
        aciAddContentToVarPacket(pkt,VAR_GPS_NUM_SATS,        &raw_pos.numsvs);
        aciAddContentToVarPacket(pkt,VAR_EST_POS_X,           &raw_pos.pos_x);
        aciAddContentToVarPacket(pkt,VAR_EST_POS_Y,           &raw_pos.pos_y);
        aciAddContentToVarPacket(pkt,VAR_EST_POS_Z,           &raw_pos.pos_z);
        aciAddContentToVarPacket(pkt,VAR_EST_ANG_X,           &raw_pos.ang_x);
        aciAddContentToVarPacket(pkt,VAR_EST_ANG_Y,           &raw_pos.ang_y);
        aciAddContentToVarPacket(pkt,VAR_EST_ANG_Z,           &raw_pos.ang_z);
        aciAddContentToVarPacket(pkt,VAR_EST_VEL_X,           &raw_pos.vel_x);
        aciAddContentToVarPacket(pkt,VAR_EST_VEL_Y,           &raw_pos.vel_y);
        aciAddContentToVarPacket(pkt,VAR_EST_VEL_Z,           &raw_pos.vel_z);
        aciAddContentToVarPacket(pkt,VAR_MAG_CAL_X,           &raw_pos.mag_x);
        aciAddContentToVarPacket(pkt,VAR_MAG_CAL_Y,           &raw_pos.mag_y);
        aciAddContentToVarPacket(pkt,VAR_MAG_CAL_Z,           &raw_pos.mag_z);
        aciSetVarPacketTransmissionRate(pkt,100);
        pkt++;
    }

    ROS_INFO("Updating packet rates");
    aciVarPacketUpdateTransmissionRates();

    ROS_INFO("Updating packet configuration");
    for (int i = 1; i < pkt; i++)
        aciSendVariablePacketConfiguration(i);

    // We have received the variables
    rcv_var = true;
}

// FLIGHT CONTROL SYSTEM ///////////////////////////////////////////////////////////

namespace platform_asctec
{
    // Class derives from all HALs
    class FlightControlSystem : 
        public hal::quadrotor::Quadrotor,
        public hal::sensor::Altimeter,
        public hal::sensor::IMU,
        public hal::sensor::Compass,
        public hal::sensor::GNSS,
        public hal::sensor::Orientation
    {

    private:
        
        // Callback timers for probing data
        ros::Timer timerHeartbeat;
        ros::Timer timerTimeout;

        // Accedd ROS node functionality
        ros::NodeHandle nh;

        // Used for timeout checking
        bool timeout;

        // Clamp a double to lie withina range
        double clamp(const double& val, const double& minval, const double& maxval)
        {
            if (val < minval) return minval;
            if (val > maxval) return maxval;
            return val;
        }

        // Called when the HAL wants an altimeter reading
        bool GetMeasurement(hal_sensor_altimeter::Data& msg)
        {
            // If the FCS hasn't initialised yet
            if (!ready) return false;
            
            // Copy over the variables
            aciSynchronizeVars();

            // Return the normalised value
            msg.t   = ros::Time::now().toSec();
            msg.z   = (double) raw_pos.pos_z / 1e3; // [ INT32] Height after data fusion (mm)
            msg.w   = (double) raw_pos.vel_z / 1e3; // [ INT32] Differential height after data fusion (mm/s)

            if (DEBUG) ROS_INFO("Received altimeter: z=%f, w=%f",msg.z,msg.w);

            // Feed the measurement to the navigation engine
            GetNavPtr()->Process(msg);

            // Success!
            return true;
        }

        // Called when the HAL wants a compass reading
        bool GetMeasurement(hal_sensor_compass::Data& msg)
        {
            // If the FCS hasn't initialised yet
            if (!ready) return false;

            // Copy over the variables
            aciSynchronizeVars();

            // Return the normalised value
            msg.t   = ros::Time::now().toSec();
            msg.x   = 0.000400000 * (double) raw_pos.mag_x; // Magnetic field strength: +-2500 = +- earth field strength
            msg.y   = 0.000400000 * (double) raw_pos.mag_y; // Magnetic field strength: +-2500 = +- earth field strength
            msg.z   = 0.000400000 * (double) raw_pos.mag_z; // Magnetic field strength: +-2500 = +- earth field strength

            if (DEBUG) ROS_INFO("Received compass: x=%f, y=%f, z=%f",msg.x,msg.y,msg.z);

            // Feed the measurement to the navigation engine
            GetNavPtr()->Process(msg);

            // Success!
            return true;
        }
        
        // Called when the HAL wants an imu reading
        bool GetMeasurement(hal_sensor_imu::Data& msg)
        {
            // If the FCS hasn't initialised yet
            if (!ready) return false;
     
            // Copy over the variables
            aciSynchronizeVars();

            // Return the scaled value (in rads/sec and m/s/s)
            msg.t   = ros::Time::now().toSec();
            msg.p   = 0.000268780 * (double) raw_imu.gyr_x;
            msg.q   = 0.000268780 * (double) raw_imu.gyr_y;
            msg.r   = 0.000268780 * (double) raw_imu.gyr_z;
            msg.du  = 0.000980665 * (double) raw_imu.acc_x;
            msg.dv  = 0.000980665 * (double) raw_imu.acc_y;
            msg.dw  = 0.000980665 * (double) raw_imu.acc_z;

            if (DEBUG) ROS_INFO("Received imu: p=%f, q=%f, r=%f, du=%f, dv=%f, dw=%f",msg.p,msg.q,msg.r,msg.du,msg.dv,msg.dw);

            // Feed the measurement to the navigation engine
            GetNavPtr()->Process(msg);

            // Get the imu
            return true;
        }
        
        // Called when the HAL wants a gnss reading
        bool GetMeasurement(hal_sensor_gnss::Data& msg)
        {
            // If the FCS hasn't initialised yet
            if (!ready) return false;

            // Copy over the variables
            aciSynchronizeVars();

            // Return the scaled value
            msg.t         = ros::Time::now().toSec();

#if USE_FUSED
            msg.latitude  = (double) raw_pos.pos_x     / 1e7;   // [ INT32] Latitude from the GPS sensor (degrees * 10^7)
            msg.longitude = (double) raw_pos.pos_y     / 1e7;   // [ INT32] Longitude from the GPS sensor (degrees * 10^7)
            msg.vel_ew    = (double) raw_pos.vel_x     / 1e3;   // [ INT32] Speed in East/West from the GPS sensor (mm/s)
            msg.vel_ns    = (double) raw_pos.vel_y     / 1e3;   // [ INT32] Speed in North/South from the GPS sensor (mm/s)
#else
            msg.latitude  = (double) raw_pos.latitude  / 1e7;   // [ INT32] Latitude from the GPS sensor (degrees * 10^7)
            msg.longitude = (double) raw_pos.longitude / 1e7;   // [ INT32] Longitude from the GPS sensor (degrees * 10^7)
            msg.vel_ew    = (double) raw_pos.vel_ew    / 1e3;   // [ INT32] Speed in East/West from the GPS sensor (mm/s)
            msg.vel_ns    = (double) raw_pos.vel_ns    / 1e3;   // [ INT32] Speed in North/South from the GPS sensor (mm/s)
#endif
            msg.altitude  = (double) raw_pos.altitude  / 1e3;   // [ INT32] Height from the GPS sensor (mm)
            msg.heading   = (double) raw_pos.heading   / 1e3;   // [ INT32] GPS Heading  (deg * 1000)
            msg.pdop      = (double) raw_pos.pdop      / 1e3;   // [UINT32] GPS position accuracy estimate  (mm)
            msg.hdop      = (double) raw_pos.hdop      / 1e3;   // [UINT32] GPS height accuracy estimate (mm)
            msg.vdop      = (double) raw_pos.vdop      / 1e3;   // [UINT32] GPS speed accuracy estimate (mm/s)
            msg.numsvs    = raw_pos.numsvs;                     // [UINT32] Number of satellites used in NAV solution (count)
   
            if (DEBUG) ROS_INFO("Received gnss: lat=%f, lon=%f, alt=%f",msg.latitude,msg.longitude,msg.altitude);


            // Feed the measurement to the navigation engine
            GetNavPtr()->Process(msg);

            // Get the GNSS
            return false;
        }
        
        // Called when the HAL wants an orientation reading
        bool GetMeasurement(hal_sensor_orientation::Data& msg)
        {
            // If the FCS hasn't initialised yet
            if (!ready) return false;

            // Copy over the variables
            aciSynchronizeVars();

            // Return the normalised values (in radians)
            msg.t     =  ros::Time::now().toSec();
            msg.roll  =  0.01745329251 * (double) raw_pos.ang_x / 1e3;              // Drift-free Roll (degree*1000, Range: -90000..+90000)
            msg.pitch = -0.01745329251 * (double) raw_pos.ang_y / 1e3;              // Drift-free Pitch (degree*1000, Range: -90000..+90000)
            msg.yaw   = -0.01745329251 * (double) raw_pos.ang_z / 1e3 + MATH_PI/2;  // Drift-free Yaw (degree*1000, Range: -90000..+90000)
            msg.p     =  0.00026878000 * (double) raw_imu.gyr_x;                    // Drift-free gyro x
            msg.q     = -0.00026878000 * (double) raw_imu.gyr_y;                    // Drift-free gyro y
            msg.r     = -0.00026878000 * (double) raw_imu.gyr_z;                    // Drift-free gyro z

            if (DEBUG) ROS_INFO("Received attitude: pitch=%f, roll=%f, yaw=%f, p=%f, q=%f, r=%f",msg.pitch,msg.roll,msg.yaw,msg.p,msg.q,msg.r);

            // Feed the measurement to the navigation engine
            GetNavPtr()->Process(msg);

            // Success!
            return true;
        }

        // Called when the HAL wants the truthful state of the platform
        void GetTruth(hal_quadrotor::State& state)
        {
            // Not valid for hardware platforms
        }

        // Called when the HAL wants to set the truthful state of the platform
        void SetTruth(const hal_quadrotor::State& state)
        {
            // Not valid for hardware platforms
        }
        
        // Called when the HAL wants to pass down some control to the platform
        double SetControl(const hal_quadrotor::Control &control)
        {
            /*
            Concerning the conversions from ACI command values to pith/roll angles and yaw speed: The mapping is as follows.
            The maximum yaw rate for the Pelican is always 200°/s (independent of the mode), with a deadzone of approx. +/- 300. 
            That means from -300 to +300 yaw command, nothing happens. From e.g. +300 to +2047, the command maps linearly 
            to 0 to 200°/s. I guess this is also the reason why the yaw rate value is limited to 1700 in your code example. 
            I guess there is more code adding or subtracting the +/- 300 deadzone later (if not, I suppose this happens 
            on the UAV internally). The maximum pitch/roll angle in Manual and Height mode is 52°. This maps linearly from
             -2047 to 2047 on ACI commands (-52° to 52°), no deadzone here. This also fits to your code snippet. In GPS mode 
             the maximum pitch/roll angle is 23°. So you need to adjust your calculations there. The angle is limited 
             because with a steeper angle, the GPS might lose too much satellites out of sight.
            */

            // Only issue control when system is ready
            // Units in = SI and RHS, units out AscTec frame
            if (rcv_cmd)
            {
#if USE_FUSED
                raw_ctl.p = clamp(-control.pitch / MATH_PI * 180.0 / 23.0 * 2047.0, -2047.0, 2047.0);      // Convert from radians -> TX vals
                raw_ctl.r = clamp( control.roll  / MATH_PI * 180.0 / 23.0 * 2047.0, -2047.0, 2047.0);       // Convert from radians -> TX vals
#else
      		    raw_ctl.p = clamp(-control.pitch / MATH_PI * 180.0 / 52.0  * 2047.0, -2047.0, 2047.0);      // Convert from radians -> TX vals
                raw_ctl.r = clamp( control.roll  / MATH_PI * 180.0 / 52.0  * 2047.0, -2047.0, 2047.0);       // Convert from radians -> TX vals
#endif
                raw_ctl.y = clamp( control.yaw   / MATH_PI * 180.0 / 200.0 * (2047.0 - 300.0) 
                          + (control.yaw > 0 ? 300.0 : -300.0), -2047.0, 2047.0); 
                raw_ctl.t =  4097.00000000000 * control.throttle;   // Convert from [0:1]   -> TX vals
                aciUpdateCmdPacket(0);
            }

            // Time st which control was applied
            return ros::Time::now().toSec();
        }

        // Error callback
        void Timeout(const ros::TimerEvent& event)
        {
            timeout = true;
        }

    public:

        // Constructor
        FlightControlSystem(double latitude, double longitude, double altitude)
        {
            // Initialise the HALs
            ROS_INFO("Initialising HALS");
            hal::quadrotor::Quadrotor::Init("~");
            hal::sensor::Altimeter::Init("~/sensor/altimeter");
            hal::sensor::Compass::Init("~/sensor/compass");
            hal::sensor::GNSS::Init("~/sensor/gnss");
            hal::sensor::IMU::Init("~/sensor/imu");
            hal::sensor::Orientation::Init("~/sensor/orientation");

            // Set the navigation origin (for converting GPS -> LTP)
            GetNavPtr()->SetOrigin(latitude, longitude, altitude);

            // Initialise the ACI engine
            ROS_INFO("Initialising ACI engine");
            aciInit();
        }

        // Try and connect to the serial
        bool Connect(std::string p, int b)
        {
            // Open serial port
            ROS_INFO("Opening serial port");
            try
            {
                serial.open(p, b);
                if (!serial.isOpen())
                    return false;
            }
            catch (boost::system::system_error e)
            {
                ROS_INFO("It appears as if the device handle does not exist");
                return false;
            }

            ROS_INFO("Configuring ACI receive callback");
            serial.setCallback(&cb_rx);
            
            ROS_INFO("Configuring ACI transmit callback");
            aciSetSendDataCallback(&cb_tx);
            
            ROS_INFO("Configuring ACI messages");
            aciSetVarListUpdateFinishedCallback(&cb_varlist);
            aciSetCmdListUpdateFinishedCallback(&cb_cmdlist);

            ROS_INFO("Configuring ACI engine rate");
            aciSetEngineRate(50,10);

            ROS_INFO("Starting ACI heartbeat");
            timerHeartbeat = nh.createTimer(
                ros::Duration(0.01),                        // duration
                &cb_hb,                                     // callback
                false                                       // object
            );
            
            ROS_INFO("Waiting for first heartbeat");
            while (!rcv_aci)
                ros::spinOnce();

            // Timeout-based variable list querying
            do
            {
                ROS_INFO("Querying variable list");
                aciGetDeviceVariablesList();
                timeout = false;
                timerTimeout = nh.createTimer(
                    ros::Duration(5.0),                   // duration
                    &FlightControlSystem::Timeout,         // callback
                    this,                                  // object
                    true                                   // oneshot
                );
                while (!rcv_var && !timeout)
                    ros::spinOnce();
                timerTimeout.stop();
                if (!timeout)
                    break;
                ROS_INFO("Timeout querying variable list");
                return false;
            }
            while (timeout);

            // Timeout-based command list querying
            do
            {
                ROS_INFO("Querying command list");
                aciGetDeviceCommandsList();
                timeout = false;
                timerTimeout = nh.createTimer(
                    ros::Duration(5.0),                   // duration
                    &FlightControlSystem::Timeout,         // callback
                    this,                                  // object
                    true                                   // oneshot
                );
                while (!rcv_cmd && !timeout)
                    ros::spinOnce();
                timerTimeout.stop();
                if (!timeout)
                    break;
                ROS_INFO("Timeout querying command list");
                return false;
            }
            while (timeout);
            
            // FCS ready
            ready = true; 
            return true;
        }
    };
}

// Main entry point of application
int main(int argc, char **argv)
{
    // Initialize the node and create a new handle
    ros::init(argc, argv, "quadrotor");
    
    // Create a new hanbdle to the flight control system
    platform_asctec::FlightControlSystem fcs(latitude, longitude, altitude);

    // Try a few times to connect
    ROS_INFO("Connecting to serial port");
    while (ros::ok())
    {
        // Connect to the flight control system via ACI
        if (fcs.Connect(port,baud))
            ros::spin();
        else
            ROS_INFO("Problems conencting to serial port. Retrying...");

        // Free up some resources for ROS
        ros::spinOnce();
    }
    
    // Everything OK
    return 0;
}
