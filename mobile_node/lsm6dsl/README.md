# Mobile node  
This is the code for the base node which uses the Disco L475 IOT01. It reads gyroscope and accelerometer data and sends to the base 
node for processing via connection-oriented Bluetooth. The system will classify movements using the sensor data from this node. 
The onboard button is used to distinguish between the teams when adding to the score.  

## Gestures  
Arm straight up (from down next to body) - Score  
Arm straight out in front - Advantage  
Arm across body - Halftime/fulltime  

**IMPORTANT** - Touching the wrong areas of the board can cause it to break/have problems, additionally the board needs to be oriented in 
a consistent position for the machine to classify the sensor values. Hold it like this:  
![Holding_pos_mobile_node](https://github.com/user-attachments/assets/2131d6ee-b23b-441b-a942-9097070700b9)  

## Flashing to Disco L475 IOT01  

Flash instructions here
