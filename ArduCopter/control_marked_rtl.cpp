#include "Copter.h"
//static Location_Class mywaypoints[] = {{363217100, 1385961150, 1500, Location_Class::ALT_FRAME_ABOVE_HOME},{363216800, 1385964920, 1500, Location_Class::ALT_FRAME_ABOVE_HOME},{363217100, 1385961150, 1500, Location_Class::ALT_FRAME_ABOVE_HOME},{363216800, 1385964920, 1500, Location_Class::ALT_FRAME_ABOVE_HOME}};
//AP_Mission mission;

static int16_t waypoint_length;
static int16_t waypoint_index=0;
static int16_t mission_finished=0;

bool Copter::marked_rtl_init(bool ignore_checks)
{
  if (position_ok() || ignore_checks) {
      land_init(true);
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

void Copter::marked_rtl_run()
{
  struct AP_Mission::Mission_Command wp_cmd={};
  //Location_Class marked_waypoints(cmd.content.location);
  waypoint_length = mission.num_commands();

  // ARM CHECK
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

  // MOTOR FULL RANGE
  // set motors to full range
  motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

// CHANGE WAYPOINT

//   if (waypoint_index > 3){
//     waypoint_index = 0;
//   }

//if(mission.num_commands()==0){
//  waypoint_length = 1;}
//else{



//if (mission.read_cmd_from_storage(waypoint_length,wp_cmd)) {
if (wp_nav->reached_wp_destination()&&mission_finished!=1){
   hal.console->printf("Index: %d\n", (int)waypoint_index);
   if (waypoint_index==0){
     waypoint_length=mission.num_commands()-1;
     hal.console->printf("START\n");
     waypoint_index++;}
   else if (waypoint_index>mission.num_commands()-1) {
     waypoint_length=0;
     hal.console->printf("LAST\n");
     mission_finished=1;
      return;}
   else{
     waypoint_length=mission.num_commands()-1-waypoint_index;
     hal.console->printf("MID\n");
     waypoint_index++;}

   mission.read_cmd_from_storage(waypoint_length,wp_cmd);
   wp_nav->set_wp_destination(wp_cmd.content.location);
   hal.console->printf("GOING HOME \n Waypoint: %d,Index: %d, Alt: %d\n", (int)waypoint_length, (int)waypoint_index, (int)wp_cmd.content.location.alt);

}
//hal.console->printf("OTW TO WAYPOINT \n");
//if (wp_nav->reached_wp_destination()){
//   waypoint_index++;
   //hal.console->printf("WAYPOINT REACHED \n");
// }

  // WAYPOINT CONTROLLER
  // run waypoint controller
  failsafe_terrain_set_status(wp_nav->update_wpnav());

  // POSITION CONTROLLER
  // call z-axis position controller (wpnav should have already updated it's alt target)
  pos_control->update_z_controller();

  // ATTITUDE CONTROLLER
  // call attitude controller
  // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
  attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(), true, get_smoothing_gain());
}
