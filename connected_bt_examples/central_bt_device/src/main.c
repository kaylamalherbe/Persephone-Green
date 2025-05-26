/**
******************************************************************************
* @file    central_bt_device.c
* @author  Lillian Kolb
* @date    26/05/2025 
* @brief   Example code for connection-oriented Bluetooth
******************************************************************************
*/

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

// #define ADDR_SIZE 17

// defining UUID for data characteristic
#define BT_UUID_DATA_CHRC_VAL BT_UUID_128_ENCODE(0xfa6d6647,0x1fcb,0x41f9,0x94ac,0x11a575e4cdc2) 
//#define BT_UUID_DATA_CHRC BT_UUID_DECLARE_128(BT_UUID_DATA_CHRC_VAL)
static const struct bt_uuid_128 data_characteristic = BT_UUID_INIT_128(BT_UUID_DATA_CHRC_VAL);

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// Service UUID in bytes, used to comapare to advertising packet
uint16_t MOBILE_UUID[] = {0xc1, 0xcd, 0xe4, 0x75, 0xa5, 0x11, 0xac, 0x94, 0xf9, 
	0x41, 0xcb, 0x1f, 0x47, 0x66, 0x6d, 0xfa};

static void start_scan(void);

// Connection variable
static struct bt_conn *default_conn;

// discovery parameters
static struct bt_gatt_discover_params discover_params;

// characteristic handle
uint16_t char_handle = 14;

// Callback function for when a Bluetooth advertising packet is found
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	// 2 1 6 11 7 c1 cd e4 75 a5 11 ac 94 f9 41 cb 1f 47 66 6d fa
	char addr_str[BT_ADDR_LE_STR_LEN];

	// check UUID, only interested in connections with correct service UUID
	if (ad->len == 21){
		for (int i = 0; i < 16; i++){
			if (MOBILE_UUID[i] != ad->data[i+5]){
				return;
			}
		}
		printk("Correct device!\n");
	} else {
		return;
	}

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	printk("Device found: %s (RSSI %d) %d\n", addr_str, rssi, ad->len);
	for(int i = 0; i < ad->len; i++) {
		printk("%x ", ad->data[i]);
	}
	printk("\n");

	// Other checks for the found device
	if (default_conn) {
		printk("default conn\n");
		return;
	}

	/* We're only interested in connectable events */
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		printk("Connectable\n");
		return;
	}

	if (bt_le_scan_stop()) {
		printk("scan stop\n");
		return;
	}

	// Create a connection to the peripheral device
	int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				BT_LE_CONN_PARAM_DEFAULT, &default_conn);
	if (err) {
		printk("Create conn to %s failed (%d)\n", addr_str, err);
		start_scan();
	}
}

// Function that starts scanning for Bluetooth advertising packets
static void start_scan(void)
{
	int err;

	/* This demo doesn't require active scan */
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}
	printk("Scanning successfully started\n");
}

// Callback function when data is read from peripheral device
static uint8_t read_periph_data_cb(struct bt_conn *conn, uint8_t err,
	struct bt_gatt_read_params *params, const void *data,
	uint16_t length) {
	
	if (data == NULL) {
		printk("It's NULL\n");
	}

	uint8_t *my_arr = ((uint8_t *)data);
	for (int i = 0; i < 8; i++) {
		printk("%x ", my_arr[i]);
	}
	printk("\n");

	return BT_GATT_ITER_STOP;
}

// Function to call when you want to read characteristic value from peripheral device
static void read_periph(void) {
	static struct bt_gatt_read_params reading_params;
	reading_params.handle_count = 1;
	reading_params.single.handle = char_handle;
	reading_params.single.offset = 0;
	reading_params.func = read_periph_data_cb;
	int err = bt_gatt_read(default_conn, &reading_params);
	if (err){
		printk("Reading error!\n");
	}
}

// Callback function when a characteristic is discovered
static uint8_t discover_func_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
	struct bt_gatt_discover_params *params) {

	printk("attr: %p\n", attr);

	if (!attr) {
		printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}
	printk("[ATTRIBUTE] handle %u\n", attr->handle);

	if (!bt_uuid_cmp(discover_params.uuid, &data_characteristic.uuid)){
		char_handle = bt_gatt_attr_value_handle(attr);
		printk("Found handle: %u\n", char_handle);
	} else {
		printk("Neither, \n");
	}

	return BT_GATT_ITER_STOP;
}

// Function that discovers the service of the peripheral device
// In this case we are only looking at 1 characteristic
static void discover_mobile_service(void) {
	printk("Discovery time\n");
	discover_params.uuid = &data_characteristic.uuid;
	discover_params.func = discover_func_cb;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
  
	int err = bt_gatt_discover(default_conn, &discover_params);
	if (err) {
	  printk("Discover failed(err %d)\n", err);
	  return;
	}
}

// This function runs when the device connects to another
static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s %u %s\n", addr, err, bt_hci_err_to_str(err));

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
		return;
	}

	if (conn != default_conn) {
		return;
	}

	printk("Connected: %s\n", addr);
	
	gpio_pin_toggle_dt(&led);
	bt_le_scan_stop();
	discover_mobile_service(); // start discovering params
}

// This function runs when the device disconnects from another
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s, reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));

	if (reason != BT_HCI_ERR_REMOTE_USER_TERM_CONN) {
		// turn off led if mobile node is disconnected
		gpio_pin_set_dt(&led, 0);
	}

	bt_conn_unref(default_conn);
	default_conn = NULL;

	start_scan();
}

// connection callback definition
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

void base_init(void)
{
	if (!gpio_is_ready_dt(&led)) {
		printk("led: device not ready\n");
	}

	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE); //GPIO_OUTPUT_INACTIVE
	if (ret < 0) {
		printk("uh oh\n");
	}

	int err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	start_scan();

}

// Thread that reads from connected device
void output(void) {
	while (1) {
		read_periph();
		k_sleep(K_MSEC(1));
	}
}


K_THREAD_DEFINE(base_init_id, STACKSIZE, base_init, NULL, NULL, NULL,
	PRIORITY, 0, 0);

K_THREAD_DEFINE(output_id, STACKSIZE, output, NULL, NULL, NULL,
	PRIORITY, 0, 0);