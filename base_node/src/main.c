/**
******************************************************************************
* @file    base_node.c
* @author  Lillian Kolb
* @date    29/04/2025 
* @brief   Final proj Base node file
******************************************************************************
*/

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

#include <UART.h>

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

// #define MOBILE_PACKET_SIZE 17
#define MOBILE_PACKET_SIZE 16

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// defining UUID for data characteristic
#define BT_UUID_DATA_CHRC_VAL BT_UUID_128_ENCODE(0xfa6d6647,0x1fcb,0x41f9,0x94ac,0x11a575e4cdc2) 
//#define BT_UUID_DATA_CHRC BT_UUID_DECLARE_128(BT_UUID_DATA_CHRC_VAL)
static const struct bt_uuid_128 data_characteristic = BT_UUID_INIT_128(BT_UUID_DATA_CHRC_VAL);

// Connection variable
static struct bt_conn *default_conn;

// discovery parameters
static struct bt_gatt_discover_params discover_params;

// characteristic handle (when loop runs too fast, must set this as the handle 
// found when running slow. Discovery function does not set handle correctly)
uint16_t char_handle = 18;

// determined whether the sensor information is send to UART or not
static char sensor_state = '1';

// Service UUID in bytes, used to comapare to advertising packet
uint16_t MOBILE_UUID[] = {0xc1, 0xcd, 0xe4, 0x75, 0xa5, 0x11, 0xac, 0x94, 0xf9, 
	0x41, 0xcb, 0x1f, 0x47, 0x66, 0x6d, 0xfa};

char MOBILE_NODE_ADDR[27] = "D8:F0:FD:6D:A0:ED (random)\0";

K_MSGQ_DEFINE(mb_msgq, MOBILE_PACKET_SIZE, 20, 4);

static void start_scan(void);

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
		return BT_GATT_ITER_STOP; // added
	}

	uint8_t *my_arr = ((uint8_t *)data);
	// for (int i = 0; i < MOBILE_PACKET_SIZE; i++) {
	// 	printk("%x ", my_arr[i]);
	// }
	// printk("\n");

	uint8_t tx_data[MOBILE_PACKET_SIZE];
	memcpy(tx_data, my_arr, MOBILE_PACKET_SIZE); //==================================== CHANGE
	if (k_msgq_put(&mb_msgq, &tx_data, K_NO_WAIT) != 0) {
		/* Queue is full, we could purge it, a loop can be
		* implemented here to keep trying after a purge.
		*/
		k_msgq_purge(&mb_msgq);
	}

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

// 
void base_init(void)
{
	if (!gpio_is_ready_dt(&led)) {
		printk("led: device not ready\n");
	}

	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE); //GPIO_OUTPUT_INACTIVE
	if (ret < 0) {
		printk("uh oh\n");
	}

	/* set MAC address */
	// use this for advertising to actuator
	int err;
	bt_addr_le_t base_addr;

	// generated using https://www.browserling.com/tools/random-mac (exception: first 2 bits must be "11")
	err = bt_addr_le_from_str("D5:16:43:05:C3:C4", "random", &base_addr);
	if (err) {
		printk("Invalid BT address (err %d)\n", err);
	}

	err = bt_id_create(&base_addr, NULL);
	if (err < 0) {
		printk("Creating new ID failed (err %d)\n", err);
	}

	// enable Bluetooth
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	start_scan();
}

// Thread that writes the sensor data read to UART 
void base_to_pc(void) {

	uint8_t sensor_data[MOBILE_PACKET_SIZE];

	printk("Mobile queue waiting...\n");
	while (1) {
		
		if (k_msgq_get(&mb_msgq, &sensor_data, K_FOREVER) == 0) {
			if (sensor_state == '1') {
				// decode the byte array received from Bluetooth
				uint32_t timestamp = (sensor_data[0] << 16) | (sensor_data[1] << 8) | sensor_data[2];
				double sensor_vals[6];

				// decoding from unsigned int to doubles
				for (int i = 0; i < 6; i++) {
					int8_t bignum = (int8_t)sensor_data[i*2 + 3]; 		// integer val
					int8_t littlenum = (int8_t)sensor_data[i*2 + 4];	// decimal val
					sensor_vals[i] = (double)bignum + ((double)littlenum)/100;
				}

				uint8_t button_state = sensor_data[15];			// state of button on board
				
				memset(&sensor_data, 0, MOBILE_PACKET_SIZE);

				// printk("%d\n", timestamp);

				char buffer[100];
				sprintf(buffer, "_sensor_ %d %.2f %.2f %.2f %.2f %.2f %.2f %d", 
					timestamp, sensor_vals[0], sensor_vals[1], sensor_vals[2], 
					sensor_vals[3], sensor_vals[4], sensor_vals[5], button_state); // if this is too slow change doubled to integers
				print_uart(buffer); 		// send to UART
			} else {
				// sensor is turned off, clear queue data
				memset(&sensor_data, 0, MOBILE_PACKET_SIZE);
				k_sleep(K_MSEC(100));
			}
		}
	}
}

// Thread that reads from connected device
void output(void) {
	k_sleep(K_MSEC(1000));
	while (1) {
		read_periph();
		k_sleep(K_MSEC(1));
	}
}

K_THREAD_DEFINE(base_init_id, STACKSIZE, base_init, NULL, NULL, NULL,
	PRIORITY - 1, 0, 0);

K_THREAD_DEFINE(base_to_pc_id, STACKSIZE, base_to_pc, NULL, NULL, NULL,
	PRIORITY, 0, 0);

K_THREAD_DEFINE(output_id, STACKSIZE, output, NULL, NULL, NULL,
	PRIORITY, 0, 0);

// Thread that reads UART and either stops UART writing or advertises classification data
void base_to_actuator(void) {
	k_sleep(K_MSEC(1000)); // wait for bt_enable

	// initial empty array for first advertisement
	struct bt_data init_array[] = {
		BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, 0x00) 
	};

	int error = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, init_array, ARRAY_SIZE(init_array), NULL, 0);
	if (error) {
		printk("Advertising start error %d\n", error);
		return;
	}

	char read_buffer[MSG_SIZE];
	uint8_t send_packet[2];

	while (1) {
		// read continuously from uart
		if (k_msgq_get(&uart_msgq, &read_buffer, K_FOREVER) == 0) {
			if (read_buffer[0] == 'c' && read_buffer[1] == ' ') {
				// this is classification information, forward to actuator node
				uint8_t class_num = read_buffer[2] - '0'; // converts char to int
				
				// find size of classification count
				uint8_t class_count_size = 0;
				for (int i = 4; i < MSG_SIZE; i++) {
					if (read_buffer[i] == '\0'){
						class_count_size = i - 4;
						break;
					}
				}
				char class_count_str[class_count_size + 1];
				memcpy(class_count_str, &read_buffer[4], class_count_size + 1);
				uint8_t class_count = atoi(class_count_str);

				// printk("classnum %d classcount %d\n", class_num, class_count);

				send_packet[0] = class_num;
				send_packet[1] = class_count;

				struct bt_data sending_array[] = {
					BT_DATA(BT_DATA_MANUFACTURER_DATA, send_packet, sizeof(send_packet))
				};
				// update advertising data with new bt_data array
				error = bt_le_adv_update_data(sending_array, ARRAY_SIZE(sending_array), NULL, 0);
				if (error) {
					printk("Advertising update error %d\n", error);
					return;
				}

			} else if (read_buffer[0] == 's' && read_buffer[1] == ' ') {
				// this turns uart on and off
				if (read_buffer[2] == '1' || read_buffer[2] == '0'){
					sensor_state = read_buffer[2];
				}
				// printk("Sensor state: %d %c [%s]\n", sensor_state, read_buffer[2], read_buffer);
			}
		}
	}
}

K_THREAD_DEFINE(base_to_actuator_id, STACKSIZE, base_to_actuator, NULL, NULL, NULL, 
	PRIORITY, 0, 0);
