#include "Copter.h"
static Location_Class mywaypoints[] = {{142583290, 1210424660, 1500, Location_Class::ALT_FRAME_ABOVE_HOME},{142578350, 1210425530, 1500, Location_Class::ALT_FRAME_ABOVE_HOME},{142583290, 1210424660, 1500, Location_Class::ALT_FRAME_ABOVE_HOME},{142578350, 1210425530, 1500, Location_Class::ALT_FRAME_ABOVE_HOME}};
static uint8_t waypoint_index;

bool Copter::p2pflight_init(bool ignore_checks)
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

void Copter::p2pflight_run()
{
  p2pflight_arm_check();

  p2pflight_motors_full_range();

  p2pflight_change_wp();

  p2pflight_waypoint_path_tracker();

  p2pflight_update_altitude();

  p2pflight_attitude_controller();
}

void Copter::p2pflight_arm_check()
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

}

void Copter::p2pflight_motors_full_range()
{

  // set motors to full range
  motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

}

void Copter::p2pflight_change_wp()
{

  if (wp_nav->reached_wp_destination()){
     waypoint_index++;
     if (waypoint_index > 3){
       waypoint_index = 0;
     }
    wp_nav->set_wp_destination(mywaypoints[waypoint_index]);

  }

}

void Copter::p2pflight_waypoint_path_tracker()
{

  // run waypoint controller
  failsafe_terrain_set_status(wp_nav->update_wpnav());

}

void Copter::p2pflight_update_altitude()
{

  // call z-axis position controller (wpnav should have already updated it's alt target)
  pos_control->update_z_controller();

}

void Copter::p2pflight_attitude_controller()
{

  // call attitude controller
  // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
  attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(), true, get_smoothing_gain());

}
