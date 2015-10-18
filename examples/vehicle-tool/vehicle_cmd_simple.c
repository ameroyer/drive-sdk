/*
 *  A simpler version of the original 
 * vehicle_cmd.c file with all event loops and 
 * callbacks removed.
 * == redefine all the functions of vehicle_cmd.c without the static definition
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <glib.h>

#include <readline/readline.h>
#include <readline/history.h>

#include <bzle/bluetooth/uuid.h>
#include <bzle/bluetooth/btio.h>
#include <bzle/gatt/att.h>
#include <bzle/gatt/gattrib.h>
#include <bzle/gatt/gatt.h>
#include <bzle/gatt/utils.h>
#include "display.h"

#include <ankidrive.h>

static GIOChannel *iochannel = NULL;
static GAttrib *attrib = NULL;
static GMainLoop *event_loop;
static GString *prompt;

static char *opt_src = NULL;
static char *opt_dst = NULL;
static char *opt_dst_type = NULL;
static char *opt_sec_level = NULL;
static int opt_psm = 0;
static int opt_mtu = 0;
static int start;
static int end;


typedef struct anki_vehicle {
  struct gatt_char read_char;
  struct gatt_char write_char;
} anki_vehicle_t;

static anki_vehicle_t vehicle;

static char *effects_by_name[] = { "STEADY", "FADE", "THROB", "FLASH", "RANDOM", NULL };
static uint8_t effect_invalid = 0xff;
static char *channels_by_name[] = { "RED", "TAIL", "BLUE", "GREEN", "FRONTL", "FRONTR", NULL };
static uint8_t channel_invalid = 0xff;

static void cmd_help(int argcp, char **argvp);

static enum state {
  STATE_DISCONNECTED,
  STATE_CONNECTING,
  STATE_CONNECTED
} conn_state;

#define error(fmt, arg...)					\
  rl_printf_simple(COLOR_RED "Error: " COLOR_OFF fmt, ## arg)

#define failed(fmt, arg...)						\
  rl_printf_simple(COLOR_RED "Command Failed: " COLOR_OFF fmt, ## arg)


// Set vehicle state
static void set_state(enum state st)
{
  conn_state = st;
}

// Function merging events_handler and handle_vehicle_msg_response
static void message_handler(const uint8_t* pdu, uint16_t len) {
  const uint8_t *data = &pdu[3];
  const uint16_t datalen = len-3;

  if (len > sizeof(anki_vehicle_msg_t)) {
    error("Invalid vehicle response\n");
    return;
  }

  const anki_vehicle_msg_t *msg = (const anki_vehicle_msg_t *)data;
  switch(msg->msg_id) {
  case ANKI_VEHICLE_MSG_V2C_PING_RESPONSE:
    {
      rl_printf_simple("[read] PING_RESPONSE\n");
      break;
    }
  case ANKI_VEHICLE_MSG_V2C_VERSION_RESPONSE:
    {
      const anki_vehicle_msg_version_response_t *m = (const anki_vehicle_msg_version_response_t *)msg;
      rl_printf_simple("[read] VERSION_RESPONSE: 0x%04x\n", m->version);
      break;
    }
  case ANKI_VEHICLE_MSG_V2C_LOCALIZATION_POSITION_UPDATE:
    {
      const anki_vehicle_msg_localization_position_update_t *m = (const anki_vehicle_msg_localization_position_update_t *)msg;
      rl_printf_simple("[read] LOCALE_UPDATE: localisationID: %02x pieceID: %02x\n", m->_reserved[0],m->_reserved[1]);

      break;
    }
  default:
    rl_printf("Received unhandled vehicle message of type 0x%02x\n", msg->msg_id);
    break;
  }
}


// Connect to vehicle
void cmd_connect_simple(char* dst, char* dst_type) {

  if (conn_state != STATE_DISCONNECTED)
    return;

  g_free(opt_dst);
  opt_dst = g_strdup(dst);

  g_free(opt_dst_type);
  opt_dst_type = g_strdup(dst_type);

  if (opt_dst == NULL) {
    error("Remote Bluetooth address required\n");
    return;
  }

  rl_printf_simple("Attempting to connect to %s\n", opt_dst);
  set_state(STATE_CONNECTING);
  //No I/O management here
  set_state(STATE_CONNECTED);
}



/*** CONTROL COMMANDS *****/

// Disconnect
void cmd_anki_vehicle_disconnect_simple()
{
  uint8_t *value;
  size_t plen;
  int handle;

  if (conn_state != STATE_CONNECTED) {
    failed("Disconnected\n");
    return;
  }

  handle = vehicle.write_char.value_handle;
  anki_vehicle_msg_t msg;
  plen = anki_vehicle_msg_disconnect(&msg);
  value = (uint8_t *)&msg;
  //gatt_write_char(attrib, handle, value, plen, NULL, NULL);
  g_free(opt_src);
  g_free(opt_dst);
  g_free(opt_sec_level);
}

// Set SDK mode
void cmd_anki_vehicle_sdk_mode_simple(int arg)
{
  uint8_t *value;
  size_t plen;
  int handle;

  if (conn_state != STATE_CONNECTED) {
    failed("Disconnected\n");
    return;
  }

  handle = vehicle.write_char.value_handle;

  anki_vehicle_msg_t msg;
  plen = anki_vehicle_msg_set_sdk_mode(&msg, arg, ANKI_VEHICLE_SDK_OPTION_OVERRIDE_LOCALIZATION);
  value = (uint8_t *)&msg;
  //gatt_write_char(attrib, handle, value, plen, NULL, NULL);
}

// Ping vehicle
void cmd_anki_vehicle_ping_simple()
{
  uint8_t *value;
  size_t plen;
  int handle;

  if (conn_state != STATE_CONNECTED) {
    failed("Disconnected\n");
    return;
  }

  handle = vehicle.write_char.value_handle;
  anki_vehicle_msg_t msg;
  plen = anki_vehicle_msg_ping(&msg);
  value = (uint8_t *)&msg;
  //gatt_write_char(attrib, handle, value, plen, NULL, NULL);
}

// U-turn
void cmd_anki_vehicle_uturn_simple()
{
  uint8_t *value;
  size_t plen;
  int handle;

  if (conn_state != STATE_CONNECTED) {
    failed("Disconnected\n");
    return;
  }

  rl_printf_simple("U-turn\n");
  handle = vehicle.write_char.value_handle;
  anki_vehicle_msg_t msg;
  plen = anki_vehicle_msg_turn_180(&msg);
  value = (uint8_t *)&msg;
  //gatt_write_char(attrib, handle, value, plen, NULL, NULL);
}

// Return vehicle's version
void cmd_anki_vehicle_get_version_simple()
{
  uint8_t *value;
  size_t plen;
  int handle;

  if (conn_state != STATE_CONNECTED) {
    failed("Disconnected\n");
    return;
  }

  handle = vehicle.write_char.value_handle;
  anki_vehicle_msg_t msg;
  plen = anki_vehicle_msg_get_version(&msg);
  value = (uint8_t *)&msg;
  //gatt_write_char(attrib, handle, value, plen, NULL, NULL);
}

// Return vehicle's location
void cmd_anki_vehicle_get_localization_position_update_simple()
{
  uint8_t *value;
  size_t plen;
  int handle;

  if (conn_state != STATE_CONNECTED) {
    failed("Disconnected\n");
    return;
  }

  handle = vehicle.write_char.value_handle;
  anki_vehicle_msg_t msg;
  plen = anki_vehicle_msg_get_localization_position_update(&msg);
  value = (uint8_t *)&msg;
  //gatt_write_char(attrib, handle, value, plen, NULL, NULL);
}

// Set vehicle's speed
void cmd_anki_vehicle_set_speed_simple(int speedd, int accell) {
  uint8_t *value;
  size_t plen;
  int handle;

  if (conn_state != STATE_CONNECTED) {
    failed("Disconnected\n");
    return;
  }

  handle = vehicle.write_char.value_handle;

  int16_t speed = (int16_t)speedd;
  int16_t accel = (int16_t)accell;
  rl_printf_simple("Setting speed to %d (accel = %d)\n", speed, accel);

  anki_vehicle_msg_t msg;
  plen = anki_vehicle_msg_set_speed(&msg, speed, accel);
  value = (uint8_t *)&msg;
  //gatt_write_char(attrib, handle, value, plen, NULL, NULL);
}

// Change lane by offset
void cmd_anki_vehicle_change_lane_simple(int hspeedd, int haccell, float offset)
{
  if (conn_state != STATE_CONNECTED) {
    failed("Disconnected\n");
    return;
  }

  int handle = vehicle.write_char.value_handle;

  int16_t hspeed = (int16_t)hspeedd;
  int16_t haccel = (int16_t)haccell;
  rl_printf_simple("changing lane at %d (accel = %d | offset = %1.2f)\n", hspeed, haccel, offset);

  anki_vehicle_msg_t msg;
  size_t plen = anki_vehicle_msg_set_offset_from_road_center(&msg, 0.0);
  //gatt_write_char(attrib, handle, (uint8_t*)&msg, plen, NULL, NULL);

  anki_vehicle_msg_t lane_msg;
  size_t lane_plen = anki_vehicle_msg_change_lane(&lane_msg, hspeed, haccel, offset);
  //gatt_write_char(attrib, handle, (uint8_t*)&lane_msg, lane_plen, NULL, NULL);
}

// Change lane by offset from center
void cmd_anki_vehicle_goto_lane_simple(int hspeedd, int haccell, float offset)
{
  if (conn_state != STATE_CONNECTED) {
    failed("Disconnected\n");
    return;
  }

  int handle = vehicle.write_char.value_handle;
  int16_t hspeed = (int16_t)hspeedd;
  int16_t haccel = (int16_t)haccell;
  rl_printf_simple("changing to lane %1.2f (speed = %d | accel = %d)\n", offset, hspeed, haccel);

  anki_vehicle_msg_t lane_msg;
  size_t lane_plen = anki_vehicle_msg_change_lane(&lane_msg, hspeed, haccel, offset);
  gatt_write_char(attrib, handle, (uint8_t*)&lane_msg, lane_plen, NULL, NULL);
}





/*
 * ADDITIONAL COMMANDS
 */
/*
  anki_vehicle_light_channel_t get_channel_by_name(const char *name)
  {
  uint8_t i;
  uint8_t channel = channel_invalid;

  if (name == NULL)
  return channel;

  uint8_t count = sizeof(channels_by_name)/sizeof(channels_by_name[0]);
  for (i = 0; i < count; i++) {
  uint8_t len = MAX(strlen(name), strlen(channels_by_name[i]));
  if (strncmp(name, channels_by_name[i], len) == 0) {
  channel = i;
  break;
  }
  }

  return channel;
  }

  anki_vehicle_light_effect_t get_effect_by_name(const char *name)
  {
  uint8_t i;
  uint8_t effect = channel_invalid;

  if (name == NULL)
  return effect;

  uint8_t count = sizeof(effects_by_name)/sizeof(effects_by_name[0]);
  for (i = 0; i < count; i++) {
  if (strncmp(name, effects_by_name[i], sizeof(effects_by_name[i])) == 0) {
  effect = i;
  break;
  }
  }

  return effect;
  }
*/

static void cmd_anki_vehicle_lights_pattern(int argcp, char **argvp)
{
  uint8_t *value;
  size_t plen;
  int handle;

  if (conn_state != STATE_CONNECTED) {
    failed("Disconnected\n");
    return;
  }

  if (argcp < 6) {
    rl_printf("Usage: %s <channel> <effect> <start> <end> <cycles_per_min>\n", argvp[0]);
    rl_printf("  channels: RED, TAIL, BLUE, GREEN, FRONTL, FRONTR\n");
    rl_printf("   effects: STEADY, FADE, THROB, FLASH, RANDOM\n");
    return;
  }

  handle = vehicle.write_char.value_handle;

  uint8_t channel = get_channel_by_name(argvp[1]);
  if (channel == channel_invalid) {
    rl_printf("Unrecognized channel: %s\n", argvp[1]);
    return;
  }

  uint8_t effect = get_effect_by_name(argvp[2]);
  if (effect == effect_invalid) {
    rl_printf("Unrecognized channel: %s\n", argvp[2]);
    return;
  }

  uint8_t start = atoi(argvp[3]);
  uint8_t end = atoi(argvp[4]);
  uint16_t cycles_per_min = atoi(argvp[5]);

  anki_vehicle_msg_t msg;
  plen = anki_vehicle_msg_lights_pattern(&msg, channel, effect, start, end, cycles_per_min);
  value = (uint8_t *)&msg;

  gatt_write_char(attrib, handle, value, plen, NULL, NULL);
}

static void vehicle_set_rgb_lights(int handle, uint8_t effect, uint8_t start_red, uint8_t end_red, uint8_t start_green, uint8_t end_green, uint8_t start_blue, uint8_t end_blue, uint16_t cycles_per_min)
{
  anki_vehicle_msg_t msg_red;
  size_t plen_red = anki_vehicle_msg_lights_pattern(&msg_red, LIGHT_RED, effect, start_red, end_red, cycles_per_min);

  anki_vehicle_msg_t msg_green;
  size_t plen_green = anki_vehicle_msg_lights_pattern(&msg_green, LIGHT_GREEN, effect, start_green, end_green, cycles_per_min);

  anki_vehicle_msg_t msg_blue;
  size_t plen_blue = anki_vehicle_msg_lights_pattern(&msg_blue, LIGHT_BLUE, effect, start_blue, end_blue, cycles_per_min);

  uint8_t *value = (uint8_t *)&msg_red;
  gatt_write_char(attrib, handle, value, plen_red, NULL, NULL);
  value = (uint8_t *)&msg_green;
  gatt_write_char(attrib, handle, value, plen_green, NULL, NULL);
  value = (uint8_t *)&msg_blue;
  gatt_write_char(attrib, handle, value, plen_blue, NULL, NULL);
}

static void cmd_anki_vehicle_engine_lights(int argcp, char **argvp)
{

  if (conn_state != STATE_CONNECTED) {
    failed("Disconnected\n");
    return;
  }

  if (argcp < 5) {
    rl_printf("Usage: %s <red> <green> <blue> <effect> <cycles per min>\n", argvp[0]);
    rl_printf("   effects: STEADY, FADE, THROB, FLASH, RANDOM\n");
    return;
  }

  float red = strtof(argvp[1], NULL);
  uint8_t r = (uint8_t)(ANKI_VEHICLE_MAX_LIGHT_INTENSITY * red);

  float green = strtof(argvp[2], NULL);
  uint8_t g = (uint8_t)(ANKI_VEHICLE_MAX_LIGHT_INTENSITY * green);

  float blue = strtof(argvp[3], NULL);
  uint8_t b = (uint8_t)(ANKI_VEHICLE_MAX_LIGHT_INTENSITY * blue);

  uint8_t effect = get_effect_by_name(argvp[4]);
  uint16_t cycles_per_min = atoi(argvp[5]);

  int handle = vehicle.write_char.value_handle;
  if (effect == EFFECT_STEADY) {
    vehicle_set_rgb_lights(handle, effect, r, r, g, g, b, b, 0);
  } else {
    vehicle_set_rgb_lights(handle, effect, 0, r, 0, g, b, 0, cycles_per_min);
  }
}

static void exchange_mtu_cb(guint8 status, const guint8 *pdu, guint16 plen,
			    gpointer user_data)
{
  uint16_t mtu;

  if (status != 0) {
    error("Exchange MTU Request failed: %s\n",
	  att_ecode2str(status));
    return;
  }

  if (!dec_mtu_resp(pdu, plen, &mtu)) {
    error("Protocol error\n");
    return;
  }

  mtu = MIN(mtu, opt_mtu);
  /* Set new value for MTU in client */
  if (g_attrib_set_mtu(attrib, mtu))
    rl_printf("MTU was exchanged successfully: %d\n", mtu);
  else
    error("Error exchanging MTU\n");
}

static void cmd_mtu(int argcp, char **argvp)
{
  if (conn_state != STATE_CONNECTED) {
    failed("Disconnected\n");
    return;
  }

  if (opt_psm) {
    failed("Operation is only available for LE transport.\n");
    return;
  }

  if (argcp < 2) {
    rl_printf("Usage: mtu <value>\n");
    return;
  }

  if (opt_mtu) {
    failed("MTU exchange can only occur once per connection.\n");
    return;
  }

  errno = 0;
  opt_mtu = strtoll(argvp[1], NULL, 0);
  if (errno != 0 || opt_mtu < ATT_DEFAULT_LE_MTU) {
    error("Invalid value. Minimum MTU size is %d\n",
	  ATT_DEFAULT_LE_MTU);
    return;
  }

  gatt_exchange_mtu(attrib, opt_mtu, exchange_mtu_cb, NULL);
}


/*
  static struct {
  const char *cmd;
  void (*func)(int argcp, char **argvp);
  const char *params;
  const char *desc;
  } commands[] = {
  { "help",		cmd_help,	"",
  "Show this help"},
  { "exit",		cmd_exit,	"",
  "Exit interactive mode" },
  { "quit",		cmd_exit,	"",
  "Exit interactive mode" },
  { "connect",		cmd_connect,	"[address [address type]]",
  "Connect to a remote device" },
  { "disconnect",		cmd_disconnect,	"",
  "Disconnect from a remote device" },
  { "mtu",		cmd_mtu,	"<value>",
  "Exchange MTU for GATT/ATT" },
  { "sdk-mode",           cmd_anki_vehicle_sdk_mode_simple,   "[on]",
  "Set SDK Mode"},
  { "ping",           cmd_anki_vehicle_ping_simple,   "",
  "Send ping message to vehicle."},
  { "get-localization-position-update",           cmd_anki_vehicle_get_localization_position_update_simple,   "",
  "Make vehicle to report its position."},
  { "get-version",           cmd_anki_vehicle_get_version_simple,   "",
  "Request vehicle software version."},
  { "set-speed",          cmd_anki_vehicle_set_speed_simple,  "<speed> <accel>",
  "Set vehicle Speed (mm/sec) with acceleration (mm/sec^2)"},
  { "change-lane",          cmd_anki_vehicle_change_lane_simple,  "<horizontal speed> <horizontal accel> <relative offset> (right(+), left(-))",
  "Change lanes at speed (mm/sec), accel (mm/sec^2) in the specified direction (offset)"},
  { "goto-lane",          cmd_anki_vehicle_goto_lane_simple,  "<horizontal speed> <horizontal accel> <absolute offset> (right(+), 0: center/init, left(-))",
  "Change to lane  given by offset at speed (mm/sec), accel (mm/sec^2)"},
  { "uturn",          cmd_anki_vehicle_uturn_simple,  "",
  "Perform U-turn"},
  { "set-lights-pattern",          cmd_anki_vehicle_lights_pattern,  "<channel> <effect> <start> <end> <cycles_per_min>",
  "Set lights pattern for vehicle LEDs."},
  { "set-engine-lights",          cmd_anki_vehicle_engine_lights,  "<red> <green> <blue> <effect> <cycles_per_min>",
  "Set the pattern for the engine lights."},
  { "vehicle-disconnect",          cmd_anki_vehicle_disconnect_simple,  "",
  "Request that the vehicle disconnect (often more reliable than disconnect)"},
  { "send-data-req",	cmd_anki_vehicle_write,	"<new value>",
  "Write data to vehicle (Request response)" },
  { "send-data",	cmd_anki_vehicle_write,	"<new value>",
  "Write data to vehicle (No response)" },
  { "read-data",	cmd_anki_vehicle_read,	"",
  "Read last message from vehicle" },
  { "info",	cmd_anki_vehicle_info,	"",
  "Read all info from vehicle that I know how" },
  { NULL, NULL, NULL}
  };

  void cmd_help(int argcp, char **argvp)
  {
  int i;

  for (i = 0; commands[i].cmd; i++)
  rl_printf("%-15s %-30s %s\n", commands[i].cmd,
  commands[i].params, commands[i].desc);
  }*/
/*
  static void parse_line(char *line_read)
  {
  char **argvp;
  int argcp;
  int i;

  if (line_read == NULL) {
  rl_printf("\n");
  cmd_exit(0, NULL);
  return;
  }

  line_read = g_strstrip(line_read);

  if (*line_read == '\0')
  goto done;

  add_history(line_read);

  if (g_shell_parse_argv(line_read, &argcp, &argvp, NULL) == FALSE)
  goto done;

  for (i = 0; commands[i].cmd; i++)
  if (strcasecmp(commands[i].cmd, argvp[0]) == 0)
  break;

  if (commands[i].cmd)
  commands[i].func(argcp, argvp);
  else
  error("%s: command not found\n", argvp[0]);

  g_strfreev(argvp);

  done:
  free(line_read);
  }*/
/*
  static gboolean prompt_read(GIOChannel *chan, GIOCondition cond,
  gpointer user_data)
  {
  if (cond & (G_IO_HUP | G_IO_ERR | G_IO_NVAL)) {
  g_io_channel_unref(chan);
  return FALSE;
  }

  rl_callback_read_char();

  return TRUE;
  }

  static char *completion_generator(const char *text, int state)
  {
  static int index = 0, len = 0;
  const char *cmd = NULL;

  if (state == 0) {
  index = 0;
  len = strlen(text);
  }

  while ((cmd = commands[index].cmd) != NULL) {
  index++;
  if (strncmp(cmd, text, len) == 0)
  return strdup(cmd);
  }

  return NULL;
  }

  static char **commands_completion(const char *text, int start, int end)
  {
  if (start == 0)
  return rl_completion_matches(text, &completion_generator);
  else
  return NULL;
  }

  static guint setup_standard_input(void)
  {
  GIOChannel *channel;
  guint source;

  channel = g_io_channel_unix_new(fileno(stdin));

  source = g_io_add_watch(channel,
  G_IO_IN | G_IO_HUP | G_IO_ERR | G_IO_NVAL,
  prompt_read, NULL);

  g_io_channel_unref(channel);

  return source;
  }

  static gboolean signal_handler(GIOChannel *channel, GIOCondition condition,
  gpointer user_data)
  {
  static unsigned int __terminated = 0;
  struct signalfd_siginfo si;
  ssize_t result;
  int fd;

  if (condition & (G_IO_NVAL | G_IO_ERR | G_IO_HUP)) {
  g_main_loop_quit(event_loop);
  return FALSE;
  }

  fd = g_io_channel_unix_get_fd(channel);

  result = read(fd, &si, sizeof(si));
  if (result != sizeof(si))
  return FALSE;

  switch (si.ssi_signo) {
  case SIGINT:
  rl_replace_line("", 0);
  rl_crlf();
  rl_on_new_line();
  rl_redisplay();
  break;
  case SIGTERM:
  if (__terminated == 0) {
  rl_replace_line("", 0);
  rl_crlf();
  g_main_loop_quit(event_loop);
  }

  __terminated = 1;
  break;
  }

  return TRUE;
  }
*/

/*
  static guint setup_signalfd(void)
  {
  GIOChannel *channel;
  guint source;
  sigset_t mask;
  int fd;

  sigemptyset(&mask);
  sigaddset(&mask, SIGINT);
  sigaddset(&mask, SIGTERM);

  if (sigprocmask(SIG_BLOCK, &mask, NULL) < 0) {
  perror("Failed to set signal mask");
  return 0;
  }

  fd = signalfd(-1, &mask, 0);
  if (fd < 0) {
  perror("Failed to create signal descriptor");
  return 0;
  }

  channel = g_io_channel_unix_new(fd);

  g_io_channel_set_close_on_unref(channel, TRUE);
  g_io_channel_set_encoding(channel, NULL, NULL);
  g_io_channel_set_buffered(channel, FALSE);

  source = g_io_add_watch(channel,
  G_IO_IN | G_IO_HUP | G_IO_ERR | G_IO_NVAL,
  signal_handler, NULL);

  g_io_channel_unref(channel);

  return source;
  }
*/
/*
  int interactive(const char *src, const char *dst,
  const char *dst_type, int psm)
  {
  guint input;
  guint signal;

  opt_sec_level = g_strdup("low");

  opt_src = g_strdup(src);
  opt_dst = g_strdup(dst);
  opt_dst_type = g_strdup(dst_type);
  opt_psm = psm;

  prompt = g_string_new(NULL);

  event_loop = g_main_loop_new(NULL, FALSE);

  input = setup_standard_input();
  signal = setup_signalfd();

  rl_attempted_completion_function = commands_completion;
  rl_erase_empty_line = 1;
  rl_callback_handler_install(get_prompt(), parse_line);

  g_main_loop_run(event_loop);

  rl_callback_handler_remove();
  cmd_disconnect(0, NULL);
  g_source_remove(input);
  g_source_remove(signal);
  g_main_loop_unref(event_loop);
  g_string_free(prompt, TRUE);

  g_free(opt_src);
  g_free(opt_dst);
  g_free(opt_sec_level);

  return 0;
  }
*/
