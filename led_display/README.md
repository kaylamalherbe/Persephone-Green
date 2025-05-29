# LED Display (Actuator node)  
This is the code for the actuator node which uses the nrf52840dk/nrf5240 and {led board name}. It reads Bluetooth packets from the base node 
and updates the LED matrix based on the received data. It uses the HUB75_driver files.  

## GPIO pin setup  


## FLashing the NRF52840DK  

To flash the NRF52840dk on zephyr, simply connect, bind and attach the board 
through the USB cable.  

You can build the program by calling:  
``west build -b nrf52840dk/nrf52840 $FILEPATH$/base_node -p``  
And you can flash the program by calling:  
``west flash``  
