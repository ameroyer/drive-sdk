/*
 * Connect to vehicle with Bluetooth address "addr" (type = "public" or "random")
 */
void cmd_connect_simple(char* dst, char* dst_type);

/*
 * Disconnect current vehicle
 */
void cmd_anki_vehicle_disconnect_simple();

/*
 * Set SDK mode to "arg"
 */
void cmd_anki_vehicle_sdk_mode_simple(int arg);

/*
 * Ping current vehicle
 */
void cmd_anki_vehicle_ping_simple();

/*
 * U-turn
 */
void cmd_anki_vehicle_uturn_simple();


/*
 * Return vehicle's version
 */
void cmd_anki_vehicle_get_version_simple();

/*
 * Returns vehicle location
 */
void cmd_anki_vehicle_get_localization_position_update_simple();

/*
 * Set vehicle's speed to "speedd" with acceleration "accell"
 */
void cmd_anki_vehicle_set_speed_simple(int speedd, int accell);

/*
 * Change lane with horizontal speed "hspeedd", horizontal accelaration "haccell" and offset "offset"
 */
void cmd_anki_vehicle_change_lane_simple(int hspeedd, int haccell, float offset);


/*
 * Change lane with horizontal speed "hspeedd", horizontal accelaration "haccell" and offset from center "offset"
 */
void cmd_anki_vehicle_gotoe_lane_simple(int hspeedd, int haccell, float offset);
