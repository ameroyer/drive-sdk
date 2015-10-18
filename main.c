/*
 * Main File for controlling the car vehicle
 * Modified version of vehicle tool with threaded execution and read from file instead of stdin
 */


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <errno.h>
#include <glib.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <bzle/bluetooth/bluetooth.h>
#include <bzle/bluetooth/hci.h>
#include <bzle/bluetooth/hci_lib.h>
#include <bzle/bluetooth/uuid.h>
#include <bzle/gatt/att.h>
#include <bzle/bluetooth/btio.h>
#include <bzle/gatt/gattrib.h>
#include <bzle/gatt/gatt.h>
#include <bzle/gatt/utils.h>

static char *opt_src = NULL;
static char *opt_dst = NULL;
static char *opt_dst_type = NULL;
static char *opt_value = NULL;
static char *opt_sec_level = NULL;
static bt_uuid_t *opt_uuid = NULL;
static int opt_handle = -1;
static int opt_mtu = 0;
static int opt_psm = 0;
static GMainLoop *event_loop;
static gboolean got_error = FALSE;
static GSourceFunc operation;
static int start = 0;

struct characteristic_data {
  GAttrib *attrib;
  uint16_t start;
  uint16_t end;
};

static void events_handler(const uint8_t *pdu, uint16_t len, gpointer user_data) 
{
  GAttrib *attrib = user_data;
  uint8_t *opdu;
  uint16_t handle, i, olen = 0;
  size_t plen;
  handle = att_get_u16(&pdu[1]);
  switch (pdu[0]) {
  case ATT_OP_HANDLE_NOTIFY:
    g_print("Notification handle = 0x%04x value: ", handle);
    break;
  case ATT_OP_HANDLE_IND:
    g_print("Indication   handle = 0x%04x value: ", handle);
    break;
  default:
    g_print("Invalid opcode\n");
    return;
  }

  for (i = 3; i < len; i++)
    g_print("%02x ", pdu[i]);

  g_print("\n");

  if (pdu[0] == ATT_OP_HANDLE_NOTIFY)
    return;

  opdu = g_attrib_get_buffer(attrib, &plen);
  olen = enc_confirmation(opdu, plen);

  if (olen > 0)
    g_attrib_send(attrib, 0, opdu, olen, NULL, NULL, NULL);
}

static GOptionEntry options[] = {
  { "adapter", 'i', 0, G_OPTION_ARG_STRING, &opt_src,
    "Specify local adapter interface", "hciX" },
  { "device", 'b', 0, G_OPTION_ARG_STRING, &opt_dst,
    "Specify remote Bluetooth address", "MAC" },
  { "addr-type", 't', 0, G_OPTION_ARG_STRING, &opt_dst_type,
    "Set LE address type. Default: random", "[public | random]"},
  { "mtu", 'm', 0, G_OPTION_ARG_INT, &opt_mtu,
    "Specify the MTU size", "MTU" },
  { "psm", 'p', 0, G_OPTION_ARG_INT, &opt_psm,
    "Specify the PSM for GATT/ATT over BR/EDR", "PSM" },
  { "sec-level", 'l', 0, G_OPTION_ARG_STRING, &opt_sec_level,
    "Set security level. Default: low", "[low | medium | high]"},
  { NULL },
};

void start_interactive(char* pipe) 
{
  GOptionContext *context;
  GOptionGroup *gatt_group, *params_group, *char_rw_group;
  GError *gerr = NULL;
  GIOChannel *chan;
  //Default
  opt_dst_type = g_strdup("random");
  opt_sec_level = g_strdup("low");
  opt_src = g_strdup("hci0");
  opt_psm = 0;
  context = g_option_context_new(NULL);
  g_option_context_add_main_entries(context, options, NULL);

  interactive_from_file(opt_src, opt_dst, opt_dst_type, opt_psm, pipe, &start);
 done:
  g_option_context_free(context);
  g_free(opt_src);
  g_free(opt_dst);
  g_free(opt_uuid);
  g_free(opt_sec_level);
  if (got_error)
    exit(EXIT_FAILURE);
  else
    exit(EXIT_SUCCESS);
}


// boson D9:81:41:5C:D4:31 [gray]
// katal D8:64:85:29:01:C0
// kourai EB:0D:D8:05:CA:1A
// rho E6:D8:52:F1:D9:43


int main(int argc, char *argv[]) {
  // Take picture
  GrabImagesFromSharedMemory(1);  

  // Create pipe an dempty it
  int fd;
  char* pipe = "communication_pipe.txt";
  fclose(fopen(pipe, "w"));

  // Run loop
  opt_dst = g_strdup("D9:81:41:5C:D4:31");
  pthread_t t;
  int ret = pthread_create (&t, NULL, (void *)start_interactive, pipe);

  // Wait for initialization - the int start is modified in the event loop run on the other thread
  while(!start) {}

  // Launch commands
  send_command(pipe, "connect");
  send_command(pipe, "set-speed 500");
  send_command(pipe, "vehicle-disconnect");
  send_command(pipe, "exit");

  // FInish
  void* retj = NULL;
  pthread_join(t, &retj);

  return 0;
}
