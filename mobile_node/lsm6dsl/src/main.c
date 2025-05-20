/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #include <zephyr/kernel.h>
 #include <zephyr/device.h>
 #include <zephyr/drivers/sensor.h>
 #include <stdio.h>
 #include <stdlib.h>
 #include <zephyr/sys/util.h>
 #include <zephyr/bluetooth/bluetooth.h>
 #include <zephyr/bluetooth/hci.h>
 
 static int print_samples;
 static int lsm6dsl_trig_cnt;
 
 static struct sensor_value accel_x_out, accel_y_out, accel_z_out;
 static struct sensor_value gyro_x_out, gyro_y_out, gyro_z_out;

 static uint8_t mfg_data[17];

 static const struct bt_data ad[] = {
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),
};
 
 #ifdef CONFIG_LSM6DSL_TRIGGER
 static void lsm6dsl_trigger_handler(const struct device *dev,
                     const struct sensor_trigger *trig)
 {
     static struct sensor_value accel_x, accel_y, accel_z;
     static struct sensor_value gyro_x, gyro_y, gyro_z;
     lsm6dsl_trig_cnt++;
 
     sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
     sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
     sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
     sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);
 
     /* lsm6dsl gyro */
     sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
     sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &gyro_x);
     sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
     sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gyro_z);
 
     if (print_samples) {
         print_samples = 0;
 
         accel_x_out = accel_x;
         accel_y_out = accel_y;
         accel_z_out = accel_z;
 
         gyro_x_out = gyro_x;
         gyro_y_out = gyro_y;
         gyro_z_out = gyro_z;

     }
 }
 #endif
 
 int main(void)
 {


    // set up bluetooth
    printk("Starting Broadcaster\n");
    int err;
    bt_addr_le_t mobile_addr;

    // set device address
    // generated using https://www.browserling.com/tools/random-mac (exception: first 2 bits must be "11")
    err = bt_addr_le_from_str("D8:F0:FD:6D:A0:ED", "random", &mobile_addr);
    if (err < 0) {
        printk("Invalid BT address (err %d)\n", err);
    }
    err = bt_id_create(&mobile_addr, NULL);
    if (err < 0) {
        printk("Creating new ID failed (err %d)\n", err);
    }
        
    //gpio_pin_toggle_dt(&led);
    // Initialize the Bluetooth Subsystem 
    err = bt_enable(NULL);
    if (err < 0) {
        printk("Bluetooth init failed (err %d)\n", err);
        // return 0;
    } else {
        printk("Bluetooth enabled\n");
    }

    printk("Bluetooth initialized\n");

    err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad), NULL, 0);
    k_msleep(100);
    if (err < 0) {
        printk("Advertising failed to start (err %d)\n", err);
        return 0;
    }

     int cnt = 0;
     char out_str[64];
     struct sensor_value odr_attr;
     const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);
 
     if (!device_is_ready(lsm6dsl_dev)) {
         printk("sensor: device not ready.\n");
         return 0;
     }
 
     /* set accel/gyro sampling frequency to 104 Hz */
     odr_attr.val1 = 104;
     odr_attr.val2 = 0;
 
     if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
                 SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
         printk("Cannot set sampling frequency for accelerometer.\n");
         return 0;
     }
 
     if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ,
                 SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
         printk("Cannot set sampling frequency for gyro.\n");
         return 0;
     }
 
 #ifdef CONFIG_LSM6DSL_TRIGGER
     struct sensor_trigger trig;
 
     trig.type = SENSOR_TRIG_DATA_READY;
     trig.chan = SENSOR_CHAN_ACCEL_XYZ;
 
     if (sensor_trigger_set(lsm6dsl_dev, &trig, lsm6dsl_trigger_handler) != 0) {
         printk("Could not set sensor type and channel\n");
         return 0;
     }
 #endif
 
     if (sensor_sample_fetch(lsm6dsl_dev) < 0) {
         printk("Sensor sample update error\n");
         return 0;
     }

    double ax = 0;
    int8_t axw = 0;
    int8_t axd = 0;

    double ay = 0;
    int8_t ayw = 0;
    int8_t ayd = 0;

    double az = 0;
    int8_t azw = 0;
    int8_t azd = 0;

    double gx = 0;
    int8_t gxw = 0;
    int8_t gxd = 0;

    double gy = 0;
    int8_t gyw = 0;
    int8_t gyd = 0;

    double gz = 0;
    int8_t gzw = 0;
    int8_t gzd = 0;
 
     while (1) {
        
        //  /* Erase previous */
        //  printk("\0033\014");
        //  printf("LSM6DSL sensor samples:\n\n");
 
        //  /* lsm6dsl accel */
        //  sprintf(out_str, "accel x:%f ms/2 y:%f ms/2 z:%f ms/2",
        //                        sensor_value_to_double(&accel_x_out),
        //                        sensor_value_to_double(&accel_y_out),
        //                 x   out_str);
 
        //  /* lsm6dsl gyro */
        //  sprintf(out_str, "gyro x:%f dps y:%f dps z:%f dps",
        //                         sensor_value_to_double(&gyro_x_out),
        //                         sensor_value_to_double(&gyro_y_out),
        //                         sensor_value_to_double(&gyro_z_out));
        //  printk("%s\n", out_str);
        //  printk("loop:%d trig_cnt:%d\n\n", ++cnt, lsm6dsl_trig_cnt);

        ax = sensor_value_to_double(&accel_x_out);
        axw = (int8_t)ax;
        axd = (int8_t)(((int16_t)(ax * 100)) % 100);

        ay = sensor_value_to_double(&accel_y_out);
        ayw = (int8_t)ay;
        ayd = (int8_t)(((int16_t)(ay * 100)) % 100);

        az = sensor_value_to_double(&accel_z_out);
        azw = (int8_t)az;
        azd = (int8_t)(((int16_t)(az * 100)) % 100);

        gx = sensor_value_to_double(&gyro_x_out);
        gxw = (int8_t)gx;
        gxd = (int8_t)(((int16_t)(gx * 100)) % 100);

        gy = sensor_value_to_double(&gyro_y_out);
        gyw = (int8_t)gy;
        gyd = (int8_t)(((int16_t)(gy * 100)) % 100);

        gz = sensor_value_to_double(&gyro_z_out);
        gzw = (int8_t)gz;
        gzd = (int8_t)(((int16_t)(gz * 100)) % 100);

        uint32_t temp_time = k_uptime_get_32();
        uint8_t time0 = temp_time >> 16;
        uint8_t time1 = temp_time >> 8;
        uint8_t time2 = temp_time;
        mfg_data[0] = time0;
		mfg_data[1] = time1;
        mfg_data[2] = time2;

        mfg_data[3] = axw;
        mfg_data[4] = axd;
        mfg_data[5] = ayw;
        mfg_data[6] = ayd;
        mfg_data[7] = azw;
        mfg_data[8] = azd;

        mfg_data[9] = gxw;
        mfg_data[10] = gxd;
        mfg_data[11] = gyw;
        mfg_data[12] = gyd;
        mfg_data[13] = gzw;
        mfg_data[14] = gzd;

        int err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
        if (err) {
            printk("Advertising update error %d\n", err);
            return err;

        }
        for (int i = 0; i < 15; i++) {
            printk("%d ", mfg_data[i]);
            if (i == 14) {
                printk("\n");
            }
        }

        printk("%.2f %.2f %.2f %.2f %.2f %.2f\n", ax, ay, az, gx, gy, gz);
        //printk("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n", );
 
        print_samples = 1;
        k_sleep(K_MSEC(100));
     }
 }
 