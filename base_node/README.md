# Base Node  

This is the code for the base node which uses the nrf52840dk/nrf5240. 
It reads Bluetooth packets from the mobile node and sends it to UART for 
processing. It also receives classification data from UART and broadcasts 
bluetooth packets to the actuator node.  

## FLashing the NRF52840DK  

To flash the NRF52840dk on zephyr, simply connect, bind and attach the board 
through the USB cable.  

You can build the program by calling:  
``west build -b nrf52840dk/nrf52840 $FILEPATH$/base_node -p``  
And you can flash the program by calling:  
``west flash``  

When the mobile node is connected to the base via Bluetooth, you should see 
LED1 on the board light up.  

``screen /dev/ttyACM0 115200`` - to connect to terminal input  