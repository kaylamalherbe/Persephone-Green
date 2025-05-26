/**
******************************************************************************
* @file    peripheral_bt_device.c
* @author  Lillian Kolb
* @date    26/05/2025 
* @brief   Example code for connection-oriented Bluetooth
******************************************************************************
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

// defining mobile node service UUID
#define BT_UUID_MOBILE_SERV_VAL BT_UUID_128_ENCODE(0xfa6d6647,0x1fcb,0x41f9,0x94ac,0x11a575e4cdc1) // randomly generated using https://www.uuidgenerator.net/
//#define BT_UUID_MOBILE_SERVICE BT_UUID_DECLARE_128(BT_UUID_MOBILE_SERV_VAL) // <- works for nrfconnect app, not for central device reading
static const struct bt_uuid_128 mobile_serv_uuid = BT_UUID_INIT_128(BT_UUID_MOBILE_SERV_VAL);

// defining UUID for data characteristic
#define BT_UUID_DATA_CHRC_VAL BT_UUID_128_ENCODE(0xfa6d6647,0x1fcb,0x41f9,0x94ac,0x11a575e4cdc2) 
//#define BT_UUID_DATA_CHRC BT_UUID_DECLARE_128(BT_UUID_DATA_CHRC_VAL)
static const struct bt_uuid_128 data_characteristic = BT_UUID_INIT_128(BT_UUID_DATA_CHRC_VAL);

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// data to send to central device, max is 20 bytes (supposedly)
// https://devzone.nordicsemi.com/f/nordic-q-a/94008/custom-notification-characteristic-not-updating
// uint8_t my_arr[8] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
uint8_t send_arr[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


// 2 1 6 11 7 c1 cd e4 75 a5 11 ac 94 f9 41 cb 1f 47 66 6d fa
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)), // saying we are using LE bluetooth not normal bluetooth
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_MOBILE_SERV_VAL), // UUID for service 
};

// This function is called when the connected device wants to read the characteristic data
static ssize_t read_data(struct bt_conn *conn, const struct bt_gatt_attr *attr,
	void *buf, uint16_t len, uint16_t offset) {
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &send_arr, sizeof(send_arr));
}

// Defining the service and characteristic of the service
BT_GATT_SERVICE_DEFINE(mobile_srv, 
	BT_GATT_PRIMARY_SERVICE(&mobile_serv_uuid), 
	BT_GATT_CHARACTERISTIC(&data_characteristic.uuid, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, read_data, NULL, NULL),
);

/* Use atomic variable, 2 bits for connection and disconnection state */
static ATOMIC_DEFINE(state, 2U);

#define STATE_CONNECTED    1U
#define STATE_DISCONNECTED 2U

// This function runs when the device connects to another
static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
	} else {
		printk("Connected\n");

		(void)atomic_set_bit(state, STATE_CONNECTED);
	}
}

// This function runs when the device disconnects from another
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));

	(void)atomic_set_bit(state, STATE_DISCONNECTED);
}

// connection callback definition
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

int main(void)
{	
	// setting LED on for troubleshooting
	if (!gpio_is_ready_dt(&led)) {
		printk("led: device not ready\n");
	}

	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE); //GPIO_OUTPUT_INACTIVE
	if (ret < 0) {
		printk("uh oh\n");
	}
	gpio_pin_set_dt(&led, 1);

	printk("Starting Observer Demo\n");

	/* Initialize the Bluetooth Subsystem */
	int err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	// start advertising, this is advertising the service uuid. Looks like:
	// 2 1 6 11 7 c1 cd e4 75 a5 11 ac 94 f9 41 cb 1f 47 66 6d fa
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}
	printk("Advertising successfully started\n");

	// this is just and example of changing data being sent to central
	uint64_t count = 0;
	while (1) {
		count++;
		for (int i = 0; i < 8; i++){
			send_arr[7-i] = ((count >> (i*8)) & 0xff);
		}
		k_sleep(K_MSEC(1));
	}

	return 0;
}