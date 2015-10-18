/*
 * Main File for controlling the car vehicle
 * Use scripts in the commands folder (simpler version of vehicle tools) and in the FlyExamples forlder (get image from shared memory)
 */


//#include "examples/vehicle-tool/vehicle_cmd_simple.c"

/***
    List of command calls
    * cmd_connect
    * cmd_anki_vehicle_read_simple(1, char* name)


    * cmd_anki_vehicle_disconnect_simple()
    * cmd_anki_vehicle_sdk_mode_simple (int mode)
    * cmd_anki_vehicle_ping_simple()

    * cmd_anki_vehicle_uturn_simple()
    * cmd_anki_vehicle_get_version_simple()
    * cmd_anki_vehicle_get_localization_position_update_simple()
    * cmd_anki_vehicle_set_speed_simple(int speed_, int accel_) (default accel = 25000) 
    * cmd_anki_vehicle_change_lane_simple(int hspeed_, int haccel_, float offset) default offset = 1
    * cmd_anki_vehicle_goto_lane_simple(int hspeed_, int haccel_, float offset) default offset = 1

///CHANGE LANESET SPEED:                 rl_printf("Usage: %s <horizo//ntal speed (mm/sec)> <horizontal accel (mm/sec^2)> <offset (mm)>\n"// , argvp[0])

////GOTOLLANE        if (argcp < 3) {
//       rl_printf("Usage: %s <horizontal speed (mm/sec)> <horizontal// accel (mm/sec^2)> <offset from center (mm)>\n", argvp[0]);

//TODO: MAC connection
*/

int main(int argc, char *argv[]) {
  cmd_anki_vehicle_sdk_mode_simple(1);
  //cmd_help(1, 0);
  return 0;
}
