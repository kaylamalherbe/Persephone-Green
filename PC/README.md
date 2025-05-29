# PC 

The PC reads information from the UART and updates a tagio web dashboard using 
JSON HTTP protocols. It interacts with the base node through a GUI. 

## Running the main program  

To run the program, call ``python $FILEPATH$/main.py``.  
Connect to the base_node shell by clicking the "Connect to shell" button.  
Send commands by writing in the "Shell Commands: " entry box and clicking the 
"Submit" button.  

## Commands  

The following are valid commands:  
``s {0/1}`` -> stop/start sensor reading  
``c {1/2/3/4}`` -> send manual classification input  
