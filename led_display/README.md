# LED Display (Actuator node)  
This is the code for the actuator node which uses the nrf52840dk/nrf5240 and {led board name}. It reads Bluetooth packets from the base node 
and updates the LED matrix based on the received data. It uses the HUB75_driver files.  

## GPIO pin setup 
nrf52840dk - Adafruit RGB Matrix Hat\
1.01 - R1\
1.02 - G1\
1.03 - B1\
1.04 - R2\
1.05 - G2\
1.06 - B2\
1.07 - A\
1.08 - B\
1.10 - C\
1.11 - D\
1.12 - CLK\
1.13 - LATCH\

## FLashing the NRF52840DK  

To flash the NRF52840dk on zephyr, simply connect, bind and attach the board 
through the USB cable.  

You can build the program by calling:  
``west build -b nrf52840dk/nrf52840 $FILEPATH$/led_display -p``  
And you can flash the program by calling:  
``west flash``  
