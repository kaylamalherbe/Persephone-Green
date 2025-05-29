
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/console/console.h>

#include <HUB75_driver.h>

#define HIGHOUT 1
#define LOWOUT 0

#define RED_ONE DT_NODELABEL(red_one)
#define GREEN_ONE DT_NODELABEL(green_one)
#define BLUE_ONE DT_NODELABEL(blue_one)
#define RED_TWO DT_NODELABEL(red_two)
#define GREEN_TWO DT_NODELABEL(green_two)
#define BLUE_TWO DT_NODELABEL(blue_two)
#define A_PIN DT_NODELABEL(a_pin)
#define B_PIN DT_NODELABEL(b_pin)
#define C_PIN DT_NODELABEL(c_pin)
#define D_PIN DT_NODELABEL(d_pin)
#define CLK_PIN DT_NODELABEL(clock_pin)
#define LATCH_PIN DT_NODELABEL(latch_pin)
#define E_PIN DT_NODELABEL(e_pin)

static const struct gpio_dt_spec R1_pin = GPIO_DT_SPEC_GET_OR(RED_ONE, gpios, {0});
static const struct gpio_dt_spec G1_pin = GPIO_DT_SPEC_GET_OR(GREEN_ONE, gpios, {0});
static const struct gpio_dt_spec B1_pin = GPIO_DT_SPEC_GET_OR(BLUE_ONE, gpios, {0});
static const struct gpio_dt_spec R2_pin = GPIO_DT_SPEC_GET_OR(RED_TWO, gpios, {0});
static const struct gpio_dt_spec G2_pin = GPIO_DT_SPEC_GET_OR(GREEN_TWO, gpios, {0});
static const struct gpio_dt_spec B2_pin = GPIO_DT_SPEC_GET_OR(BLUE_TWO, gpios, {0});
static const struct gpio_dt_spec A_pin = GPIO_DT_SPEC_GET_OR(A_PIN, gpios, {0});
static const struct gpio_dt_spec B_pin = GPIO_DT_SPEC_GET_OR(B_PIN, gpios, {0});
static const struct gpio_dt_spec C_pin = GPIO_DT_SPEC_GET_OR(C_PIN, gpios, {0});
static const struct gpio_dt_spec D_pin = GPIO_DT_SPEC_GET_OR(D_PIN, gpios, {0});
static const struct gpio_dt_spec CLK_pin = GPIO_DT_SPEC_GET_OR(CLK_PIN, gpios, {0});
static const struct gpio_dt_spec LATCH_pin = GPIO_DT_SPEC_GET_OR(LATCH_PIN, gpios, {0});
static const struct gpio_dt_spec E_pin = GPIO_DT_SPEC_GET_OR(E_PIN, gpios, {0});

void init_pins(void) {

    //printf("HI\n");

    for (int i = 0; i < 16; i++) {
        screen_map[i][15] = 1;
        screen_map[i][16] = 1;
    }

    int err;
    err = gpio_pin_configure_dt(&R1_pin, GPIO_OUTPUT_INACTIVE);
    err = gpio_pin_configure_dt(&G1_pin, GPIO_OUTPUT_INACTIVE);
    err = gpio_pin_configure_dt(&B1_pin, GPIO_OUTPUT_INACTIVE);
    err = gpio_pin_configure_dt(&R2_pin, GPIO_OUTPUT_INACTIVE);
    err = gpio_pin_configure_dt(&G2_pin, GPIO_OUTPUT_INACTIVE);
    err = gpio_pin_configure_dt(&B2_pin, GPIO_OUTPUT_INACTIVE);
    err = gpio_pin_configure_dt(&A_pin, GPIO_OUTPUT_INACTIVE);
    err = gpio_pin_configure_dt(&B_pin, GPIO_OUTPUT_INACTIVE);
    err = gpio_pin_configure_dt(&C_pin, GPIO_OUTPUT_INACTIVE);
    err = gpio_pin_configure_dt(&D_pin, GPIO_OUTPUT_INACTIVE);
    err = gpio_pin_configure_dt(&CLK_pin, GPIO_OUTPUT_INACTIVE);
    err = gpio_pin_configure_dt(&LATCH_pin, GPIO_OUTPUT_INACTIVE);
    err = gpio_pin_configure_dt(&E_pin, GPIO_OUTPUT_INACTIVE);
    if (err != 0) {
        printf("Configuring pin failed: %d\n", err);
    }

    gpio_pin_set_dt(&R1_pin, LOWOUT);
    gpio_pin_set_dt(&G1_pin, LOWOUT);
    gpio_pin_set_dt(&B1_pin, LOWOUT);
    gpio_pin_set_dt(&R2_pin, LOWOUT);
    gpio_pin_set_dt(&G2_pin, LOWOUT);
    gpio_pin_set_dt(&B2_pin, LOWOUT);
    gpio_pin_set_dt(&A_pin, LOWOUT);
    gpio_pin_set_dt(&B_pin, LOWOUT);
    gpio_pin_set_dt(&C_pin, LOWOUT);
    gpio_pin_set_dt(&D_pin, LOWOUT);
    gpio_pin_set_dt(&CLK_pin, LOWOUT);
    gpio_pin_set_dt(&LATCH_pin, 0);
    gpio_pin_set_dt(&E_pin, 0);

}
    

void refresh_screen(void) {

    for (int i = 0; i < 16; i++) {
        
        // if in the first 12 rows, choose corresponding row from bitmap

        gpio_pin_set_dt(&A_pin, (i >> 0) & 1);
        gpio_pin_set_dt(&B_pin, (i >> 1) & 1);
        gpio_pin_set_dt(&C_pin, (i >> 2) & 1);
        gpio_pin_set_dt(&D_pin, (i >> 3) & 1);

        //printk("%d %d %d %d\n", (i >> 3) & 1, (i >> 2) & 1, (i >> 1) & 1, (i >> 0) & 1);

        for (int w = 0; w < 32; w++) {
            // if in first 12 columns, sel from the bitmap, else set to 0
            
            // if (w < 12 && i < 12) {
            //     sel = digit_bitmaps[6][i][w];
            // } else {
            //     sel = 0;
            // }
            gpio_pin_set_dt(&R1_pin, screen_map[i][w]);
            gpio_pin_set_dt(&G1_pin, 0);
            gpio_pin_set_dt(&B1_pin, 0);
            gpio_pin_set_dt(&R2_pin, screen_map[i + 16][w]);
            gpio_pin_set_dt(&G2_pin, 0);
            gpio_pin_set_dt(&B2_pin, 0);
            //printk("%d", sel);

            gpio_pin_set_dt(&CLK_pin, HIGHOUT);
            gpio_pin_set_dt(&CLK_pin, LOWOUT);
        }

        //printk("\n");

        // Latch row
        gpio_pin_set_dt(&LATCH_pin, HIGHOUT);
        gpio_pin_set_dt(&LATCH_pin, LOWOUT);

        k_busy_wait(300); 

        //gpio_pin_set_dt(&E_pin, LOWOUT);

        //k_msleep(2); // Adjust for desired refresh rate

    }
}


void update_score(int home_score, int away_score) {
    for (int h = 0; h < 12; h++) {
        for (int w = 0; w < 12; w++) {
            screen_map[h + 2][w + 2] = digit_bitmaps[home_score][h][w];
            screen_map[h + 2][w + 18] = digit_bitmaps[away_score][h][w];
        }
    }
}

void print_try(void) {
    for (int h = 0; h < 16; h++) {
        for (int w = 0; w < 32; w++) {
            screen_map[h+16][w] = try_bm[h][w];
        }
    }
}

void print_teams(void) {
    for (int h = 0; h < 16; h++) {
        for (int w = 0; w < 32; w++) {
            screen_map[h+16][w] = teams_bm[h][w];
        }
    }
}

void print_fulltime(void) {
    for (int h = 0; h < 16; h++) {
        for (int w = 0; w < 32; w++) {
            screen_map[h+16][w] = fulltime_bm[h][w];
        }
    }
}

void print_halftime(void) {
    for (int h = 0; h < 16; h++) {
        for (int w = 0; w < 32; w++) {
            screen_map[h+16][w] = halftime_bm[h][w];
        }
    }
}

void print_adv(void) {
    for (int h = 0; h < 16; h++) {
        for (int w = 0; w < 32; w++) {
            screen_map[h+16][w] = advantage_bm[h][w];
        }
    }
}