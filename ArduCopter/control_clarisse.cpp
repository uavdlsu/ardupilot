#include "Copter.h"
static Location_Class mywaypoints[] = {{363217100, 1385961150, 1500, Location_Class::ALT_FRAME_ABOVE_HOME},{363216800, 1385964920, 1500, Location_Class::ALT_FRAME_ABOVE_HOME},{363217100, 1385961150, 1500, Location_Class::ALT_FRAME_ABOVE_HOME},{363216800, 1385964920, 1500, Location_Class::ALT_FRAME_ABOVE_HOME}};
static uint8_t waypoint_index;

bool Copter::clarisse_init(bool ignore_checks)
{
  if (position_ok() || ignore_checks) {
      // initialise yaw
      set_auto_yaw_mode(get_default_auto_yaw_mode(false));
      // start in position control mode
      // initialise waypoint and spline controller
      wp_nav->wp_and_spline_init();

      // initialise wpnav to stopping point
      Vector3f stopping_point;
      wp_nav->get_wp_stopping_point(stopping_point);

      // no need to check return status because terrain data is not used
      wp_nav->set_wp_destination(stopping_point, false);

      // initialise yaw
      set_auto_yaw_mode(get_default_auto_yaw_mode(false));
      return true;
  }else{
      return false;
  }
}

void Copter::clarisse_run()
{
  // if not auto armed or motors not enabled set throttle to zero and exit immediately
  if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
      // call attitude controller
      attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
      attitude_control->set_throttle_out(0,false,g.throttle_filt);
#else
      motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
      // multicopters do not stabilize roll/pitch/yaw when disarmed
      attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
      return;
  }

  // set motors to full range
  motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

if (wp_nav->reached_wp_destination()){
   waypoint_index++;
   if (waypoint_index > 3){
     waypoint_index = 0;
   }
  wp_nav->set_wp_destination(mywaypoints[waypoint_index]);
}
  // run waypoint controller
  failsafe_terrain_set_status(wp_nav->update_wpnav());

  // call z-axis position controller (wpnav should have already updated it's alt target)
  pos_control->update_z_controller();

  // call attitude controller
  // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
  attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(), true, get_smoothing_gain());
}
