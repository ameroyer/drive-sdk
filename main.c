/*
 * Main File for controlling the car vehicle
 * Use scripts in the commands folder (simpler version of vehicle tools) and in the FlyExamples forlder (get image from shared memory)
 */


#include "examples/vehicle-tool/vehicle_cmd_simple.c"

/***
List of command calls
* Note to check: some command are already defined in non static somewher, eg cmnd_anki_vehicle_sdk_mode
*///

//TODO: MAC connection
int main(int argc, char *argv[])
{
  cmd_anki_vehicle_sdk_mode(1, 0);
	  //cmd_help(1, 0);
    	return 0;
}
