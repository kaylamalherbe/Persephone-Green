/**
******************************************************************************
* @file    base_node.c
* @author  Lillian Kolb
* @date    23/04/2025 
* @brief   Prac 3 Base node file
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
#include <zephyr/bluetooth/uuid.h>

#include <UART.h>

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#define MOBILE_PACKET_SIZE 17

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

char MOBILE_NODE_ADDR[27] = "D8:F0:FD:6D:A0:ED (random)\0";

K_MSGQ_DEFINE(mb_msgq, MOBILE_PACKET_SIZE, 20, 4);

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	// print_uart(addr_str);

	// only read data from mobile node
	if (!strcmp(addr_str, MOBILE_NODE_ADDR)){
		// for (int i =0; i < ad->len; i++) {
		// 	printk("%x ", ad->data[i]);
		// }
		// printk("\n");
		uint8_t* rec_data = ad->data;
		uint8_t tx_data[MOBILE_PACKET_SIZE];
		memcpy(tx_data, rec_data, MOBILE_PACKET_SIZE); //==================================== CHANGE
		if (k_msgq_put(&mb_msgq, &tx_data, K_NO_WAIT) != 0) {
			/* Queue is full, we could purge it, a loop can be
			* implemented here to keep trying after a purge.
			*/
			k_msgq_purge(&mb_msgq);
		}
	}
}

static void start_scan(void)
{
	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_PASSIVE,
		.options    = BT_LE_SCAN_OPT_NONE,	// does not filter duplicate packets
		.interval   = BT_GAP_SCAN_FAST_INTERVAL,
		.window     = BT_GAP_SCAN_FAST_WINDOW,
	};

	/* This demo doesn't require active scan */
	int err = bt_le_scan_start(&scan_param, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

void base_init(void)
{
	if (!gpio_is_ready_dt(&led)) {
		printk("led: device not ready\n");
	}

	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE); //GPIO_OUTPUT_INACTIVE
	if (ret < 0) {
		printk("uh oh\n");
	}

	gpio_pin_set_dt(&led, 1); // turns on led 1 on the board

	/* set MAC address */
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

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}
	printk("Bluetooth initialized\n");

	start_scan();
}

void base_to_pc(void) {

	uint8_t sensor_data[MOBILE_PACKET_SIZE];

	printk("Mobile queue waiting...\n");
	while (1) {
		
		if (k_msgq_get(&mb_msgq, &sensor_data, K_FOREVER) == 0) {

			// for (int i = 0; i < MOBILE_PACKET_SIZE; i++) {
			// 	printk("%x ", sensor_data[i]);
			// }
			// printk("\n");

			uint32_t timestamp = (sensor_data[2] << 16) | (sensor_data[3] << 8) | sensor_data[4];
			double sensor_vals[6];

			for (int i = 0; i < 6; i++) {
				int8_t bignum = (int8_t)sensor_data[i*2 + 5];
				int8_t littlenum = (int8_t)sensor_data[i*2 + 6];
				sensor_vals[i] = (double)bignum + ((double)littlenum)/100;
			}
			
			memset(&sensor_data, 0, MOBILE_PACKET_SIZE);

			// printk("%d\n", timestamp);

			char buffer[100];
			sprintf(buffer, "_sensor_ %d %.2f %.2f %.2f %.2f %.2f %.2f", 
				timestamp, sensor_vals[0], sensor_vals[1], sensor_vals[2], 
				sensor_vals[3], sensor_vals[4], sensor_vals[5]); // if this is too slow change doubled to integers
			print_uart(buffer);
		}
	}
}

K_THREAD_DEFINE(base_init_id, STACKSIZE, base_init, NULL, NULL, NULL,
	PRIORITY, 0, 0);

K_THREAD_DEFINE(base_to_pc_id, STACKSIZE, base_to_pc, NULL, NULL, NULL,
	PRIORITY, 0, 0);

void base_to_actuator(void) {

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

	while (1) {
		// read continuously from uart
		if (k_msgq_get(&uart_msgq, &read_buffer, K_FOREVER) == 0) {
			// gpio_pin_toggle_dt(&led);
			// currently code is simply writing back what it received
			char send_buffer[100];

			uint64_t time_ms = k_uptime_get();
			uint32_t time_sec = time_ms/1000;

			sprintf(send_buffer, "%dsec %lldms %llx %s", time_sec, time_ms%1000, time_ms, read_buffer);
			print_uart(send_buffer);
			// print_uart(read_buffer);
		}
	}
}

K_THREAD_DEFINE(base_to_actuator_id, STACKSIZE, base_to_actuator, NULL, NULL, NULL, 
	PRIORITY, 0, 0);

