

#include <HUB75_driver.h>

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



K_MSGQ_DEFINE(my_msgq, sizeof(uint8_t), 20, 4);

int scoreH = 0;
int scoreA = 0;
int classcount = 0;
int half_count = 0;

// in classification
// half is 1
// team 1 score is 2
// scrum is 3
// team 2 score is 4



char BASE_NODE_ADDR[27] = "D5:16:43:05:C3:C4 (random)\0";

static bool reading_parse_cb(struct bt_data *data, void *user_data) {
    if ((data->type) == BT_DATA_MANUFACTURER_DATA) {
        // printing received data
        
        // for (int i = 0; i < data->data_len; i++) {
        //     printk("%d ", data->data[i]);
        // }
        // printk("\n");

        // now decode your received data here

        if (data->data[0] != 0) {
            uint8_t classification = data->data[0];        // classification from machine
            uint8_t class_count = data->data[1];        // total count of gestures (use to check if new gesture in)
            // ^ class_count, have a prev_count variable and check if they are same or different. 
            // This is cause you'll be reading multiple of the same packets, so you need to know
            // when the matrix actually needs to be updated

            // USE THESE VALUES TO UPDATE LED MATRIX

            // printk("%d %d\n", classification, class_count);

            if (class_count != classcount) {
                
                printk("%d %d\n", classification, class_count);

                //printk("hi\n");
                classcount = class_count;

                //update_score(scoreH, scoreA);

                printk("classification sending: %d\n", classification);

                if (k_msgq_put(&my_msgq, &classification, K_NO_WAIT) != 0) {
                    /* Queue is full, we could purge it, a loop can be
                    * implemented here to keep trying after a purge.
                    */
                    k_msgq_purge(&my_msgq);
                }

                //k_msleep(10);

                
            }
        }

        
        
    }

    return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi,
    uint8_t type, struct net_buf_simple *ad)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    //printk("Device found: %s (RSSI %d)\n", addr_str, rssi);

    if (!strcmp(addr_str, BASE_NODE_ADDR)){
        bt_data_parse(ad, reading_parse_cb, (void *)addr);
        //printk("found\n");
    }
}

void init_read(void) {
    k_sleep(K_MSEC(1000));
    int err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    struct bt_le_scan_param scan_param = {
        .type       = BT_LE_SCAN_TYPE_PASSIVE,
        .options    = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval   = BT_GAP_SCAN_FAST_INTERVAL,
        .window     = BT_GAP_SCAN_FAST_WINDOW,
    };

    err = bt_le_scan_start(&scan_param, device_found);
    if (err) {
        printk("Start scanning failed (err %d)\n", err);
        return;
    }
    printk("Started scanning.......\n");

    //while(1) {k_sleep(K_FOREVER);}
}


void main_thread(void) {
    k_sleep(K_MSEC(1000));

    init_pins();

    update_score(0, 0);
    print_teams();

    while(1) {
        refresh_screen();
        //k_sleep(K_MSEC(10));
    }

    return;
}

void update_screen_thread(void) {
    uint8_t rx_data;
    //printk("hi\n");
    while (1) {
        // read continuously from uart
        printk("reading\n");
        if (k_msgq_get(&my_msgq, &rx_data, K_FOREVER) == 0) {

            printk("receiving: %d\n", rx_data);

            switch(rx_data) {
                case 1:
                    if (half_count%2) {
                        scoreH = 0;
                        scoreA = 0;
                        update_score(scoreH, scoreA);
                        print_fulltime();
                    } else {
                        print_halftime();
                    }
                    half_count++;
                    break;
                case 2:
                    scoreH++;
                    update_score(scoreH, scoreA);
                    print_try();
                    break;
                case 3:
                    print_adv();
                    break;
                case 4:
                    scoreA++;
                    update_score(scoreH, scoreA);
                    print_try();
                    break;

             }

            k_msleep(3000);
            print_teams();
            memset(&rx_data, 0, sizeof(uint8_t));

        }

        
        
    }
}

K_THREAD_DEFINE(update_screen_id, 1024, update_screen_thread, NULL, NULL, NULL, 
    6, 0, 0);


// make sure to define stack and priority
K_THREAD_DEFINE(init_read_id, 512, init_read, NULL, NULL, NULL, 
    7, 0, 0);

K_THREAD_DEFINE(main_thread_id, 512, main_thread, NULL, NULL, NULL, 7, 0, 0);

