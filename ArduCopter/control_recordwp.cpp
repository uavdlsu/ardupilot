#include "Copter.h"

bool Copter::recordwp_init(bool ignore_checks)
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

// auto_run - runs the auto controller
//      should be called at 100hz or more
//      relies on run_autopilot being called at 10hz which handles decision making and non-navigation related commands
void Copter::recordwp_run()
{
  // if not auto armed or motors not enabled set throttle to zero and exit immediately
  if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
      motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
      // multicopters do not stabilize roll/pitch/yaw when disarmed
      attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
}

  // set motors to full range
  motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);


  wp_nav->set_wp_destination(destination, false);
  // run waypoint controller
  failsafe_terrain_set_status(wp_nav->update_wpnav());

  // call z-axis position controller (wpnav should have already updated it's alt target)
  pos_control->update_z_controller();

  // call attitude controller
  // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
  attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(), true, get_smoothing_gain());
}
