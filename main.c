/*
 * Main File for controlling the car vehicle
 * Use scripts in the commands folder (simpler version of vehicle tools) and in the FlyExamples forlder (get image from shared memory)
 */


#include "examples/vehicle-tool/vehicle_cmd_simple.h"

// boson D9:81:41:5C:D4:31
// katal D8:64:85:29:01:C0
// kourai EB:0D:D8:05:CA:1A
// rho E6:D8:52:F1:D9:43

int main(int argc, char *argv[]) {
  cmd_connect_simple("D8:64:85:29:01:C0", "random");
  cmd_anki_vehicle_sdk_mode_simple(1);
  cmd_anki_vehicle_set_speed_simple(500, 25000);
  cmd_anki_vehicle_disconnect_simple();
  //cmd_help(1, 0);
  return 0;
}
