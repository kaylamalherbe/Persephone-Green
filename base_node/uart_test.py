import serial
import time
from tkinter import *
from tkinter import scrolledtext
import threading
import json
import requests
from datetime import datetime
import pandas as pd
import sys
import os

read_str = ""

CSV_SIZE = 1200
data_count = 0

def reading_thread():
    try:
        global ser
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        count = 0
        global data_count
        data = []
        dirname = os.path.dirname(__file__)
        print(dirname)
        movement = 'scrumadv'
        csv_name = dirname + '/data_' + movement + '.csv'
        
        # create dataframe
        df = pd.DataFrame()
        while (1):
            try: 
                line = ser.readline()   # read a '\n' terminated line
                received = line.decode('utf-8').strip()
                split_str = received.split(" ")

                global read_str
                if (received != "" and split_str[0] == "_sensor_"):
                    read_str += received + "\n"
                    count += 1

                    split_str.pop(0)
                    data.append(split_str)
                    data_count += 1
                
                if (count > 30):
                    read_str = ""
                    count = 0

                if (data_count >= CSV_SIZE):
                    print("Reached data limit")
                    # fill dataframe headings Ax, Ay, Az, Gz, Gy, Gz, Category
                    df['Time'] = [data[i][0] for i in range(0, len(data))]
                    df['Ax'] = [data[i][1] for i in range(0, len(data))]
                    df['Ay'] = [data[i][2] for i in range(0, len(data))]
                    df['Az'] = [data[i][3] for i in range(0, len(data))]
                    df['Gx'] = [data[i][4] for i in range(0, len(data))]
                    df['Gy'] = [data[i][5] for i in range(0, len(data))]
                    df['Gz'] = [data[i][6] for i in range(0, len(data))]
                    df['Category'] = [data[i][7] for i in range(0, len(data))]
                    df.to_csv(csv_name, index=False)
                    print("saved to csv " + csv_name)
                    break

            except serial.serialutil.SerialException:
                sys.exit("Port connection broken")
                break
        
    except serial.serialutil.SerialException:
        print("Unable to connect to port")
    

# print("Starting GUI...")
main = Tk()
main.title("User input GUI")
main.geometry("600x650")

top_frame = Frame(main, height = 150, width = 600) #bg="red"
top_frame.grid(row=0, column=0)

bottom_frame = Frame(main, height = 500, width = 600) #bg="blue"
bottom_frame.grid(row=1, column=0)

def send_uart():
    write_str = tempVar.get() + "\n"# received input from user
    if (ser != None):
        try: 
            ser.write(write_str.encode('utf-8'))
        except serial.serialutil.SerialException:
            print("Unable to write to port")
    else:
        print("Port is not connected")
    
    input_box.delete(0, END)

# connect to shell
def uart_read_thread(): 
    tempthread = threading.Thread(target=reading_thread,
                              args=())
    tempthread.daemon = True
    tempthread.start()
    shell_connect_Button.config(state=DISABLED)

def update():
    outputText.config(state=NORMAL)
    outputText.delete("1.0", END)
    outputText.insert(END, read_str)
    outputText.config(state=DISABLED)

    dataLabel.config(text="Data count: " + str(data_count))

    main.after(100, update) # run itself again after 100 ms

varLabel = Label(top_frame, text = "Shell Commands: ")
varLabel.place(x = 50, y = 30)

tempVar = StringVar()

input_box = Entry(top_frame, textvariable = tempVar, width = 30, state = NORMAL)
input_box.place(x = 50, y = 50)
input_Button = Button(top_frame, text = "Submit", state = NORMAL, command=send_uart)
input_Button.place(x = 50, y = 80)

shell_connect_Button = Button(top_frame, text = "Connect to shell", state = NORMAL, command=uart_read_thread)
shell_connect_Button.place(x = 450, y = 20)

dataLabel = Label(top_frame, text = "Data count: ")
dataLabel.place(x = 400, y = 60)

outputText = scrolledtext.ScrolledText(bottom_frame, height = 25, width = 60, state = NORMAL)
outputText.place(x = 50, y = 20)

update()
# Closes mainloop for window
main.mainloop()