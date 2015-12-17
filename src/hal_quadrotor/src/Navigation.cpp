#include <hal_quadrotor/Navigation.h>

#define DEBUG false
#define PI 3.14159265359
#define MASK_GPS  0b00000001
#define MASK_ALT  0b00000010
#define MASK_MAG  0b00000100
#define MASK_IMU  0b00001000
#define MASK_ROT  0b00010000
#define MASK_ALL  0b00011111

using namespace hal::quadrotor;

Navigation::Navigation() : data(0x0), ready(false),
  wgs84_ecef(
    GeographicLib::Constants::WGS84_a(), 
    GeographicLib::Constants::WGS84_f()
  )
{
  // Do nothing
}

// Rotation from body to navigation frame void Navigation::n2b(double rot[3], double vec[3]) {
void Navigation::n2b(double rot[3], double vec[3])
{
    double t[3], c[3], s[3];
    for (int i = 0; i < 3; i++)
    {
        t[i] = vec[i];
        c[i] = cos(rot[i]);
        s[i] = sin(rot[i]);
    }
    vec[0] =                (c[1]*c[2])*t[0] +                (c[1]*s[2])*t[1] -      s[1]*t[2];
    vec[1] = (s[1]*s[0]*c[2]-s[2]*c[0])*t[0] + (s[1]*s[0]*s[2]+c[2]*c[0])*t[1] + c[1]*s[0]*t[2];
    vec[2] = (s[1]*c[0]*c[2]-s[2]*s[0])*t[0] + (s[1]*c[0]*s[2]+c[2]*s[0])*t[1] + c[1]*c[0]*t[2];
}

void Navigation::Reset()
{
  // Do nothing
}

bool Navigation::GetState(hal_quadrotor::State &msg)
{
  // Copy the state
  msg = state;
 
  // Important - the measurements of velocity are in the nav frame,
  // and we need them in the body frame for the controllers to work
  double rot[] = {state.roll, state.pitch, state.yaw};
  double vel[] = {vel_x,      vel_y,       state.w  };
  n2b(rot,vel);
  msg.u = vel[0];
  msg.v = vel[1];
  msg.w = vel[2];

  // IMPORTANT - the latiude and longitude are in WGS84 coordinates,
  // so we need to convert them to LTP pre-return 
  double x, y, z;
  wgs84_enu.Forward(
    pos_y, 
    pos_x, 
    altitude,
    x,
    y,
    z
  );
  msg.x = x;
  msg.y = y;
  // Ignore z, as barometric is better

  // Set the time
  msg.t = ros::Time::now().toSec();
  
  // Only successful if all data has been received
  return (data==MASK_ALL);
}

void Navigation::SetState(const hal_quadrotor::State &msg)
{
  state   = msg;
  data    = MASK_ALL;
  state.t = ros::Time::now().toSec();
}

void Navigation::Process(const hal_sensor_altimeter::Data &msg)
{
  state.z = msg.z;
  state.w = msg.w;

  // Update mask
  data |= MASK_ALT;

  // Information
  if (DEBUG) ROS_INFO("A %f-- POS %f VEL: %f", msg.t, state.z, state.w);
}

void Navigation::Process(const hal_sensor_compass::Data &msg)
{
  // Update mask
  data |= MASK_MAG;

  // Information
  if (DEBUG) ROS_INFO("C %f -- MAG %f %f %f", msg.t, msg.x, msg.y, msg.z);
}

void Navigation::Process(const hal_sensor_imu::Data &msg)
{
  // Update mask
  data |= MASK_IMU;

  // Information
  if (DEBUG) ROS_INFO("I %f -- ANG %f %f %f ACC %f %f %f", msg.t, msg.p, msg.q, msg.r, msg.du, msg.dv, msg.dw);
}

void Navigation::Process(const hal_sensor_gnss::Data &msg)
{
  // If the LTP origin was set
  if (ready)
  { 
    // Set the values (in n-frame for now...)
    vel_x    = msg.vel_ew;
    vel_y    = msg.vel_ns;

    // Save absolute altitude for projection in GetState()
    pos_x    = msg.longitude; 
    pos_y    = msg.latitude;
    altitude = msg.altitude;

    // Update mask
    data |= MASK_GPS;

    // Information
    if (DEBUG) ROS_INFO("G %f -- POS %f %f VEL: %f %f", msg.t, state.x, state.y, state.u, state.v);
  }

}

void Navigation::Process(const hal_sensor_orientation::Data &msg)
{
  state.roll  =  msg.roll;
  state.pitch =  msg.pitch;
  state.yaw   =  msg.yaw;
  state.p     =  msg.p;
  state.q     =  msg.q;
  state.r     =  msg.r;

  // Update mask
  data |= MASK_ROT;

  // Information
  if (DEBUG) ROS_INFO("O %f -- ANG %f %f %f VEL: %f %f %f", msg.t, 
    state.roll, state.pitch, state.yaw, state.p, state.q, state.r);

}

void Navigation::SetOrigin(double latitude, double longitude, double altitude)
{
  // Convert from WGS84 to LTP coordinates
  wgs84_enu = GeographicLib::LocalCartesian(
    latitude, 
    longitude, 
    altitude, 
    wgs84_ecef
  ); 

  // We are notw ready to capture GPS data
  ready = true;
}
