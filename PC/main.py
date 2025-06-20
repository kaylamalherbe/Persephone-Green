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
import numpy as np
import joblib
import warnings
warnings.filterwarnings('ignore')

# for the dashboard Https conenciton
headers = {
        "Device-Token": "1b5e7261-4b25-4360-96e7-1f2957aa9ac2",
        "Content-Type": "application/json"
    }

payload = [
    {"variable": "Classification", "value": 0},
    {"variable": "Score1", "value": 0, "unit": ""},
    {"variable": "Score2", "value": 0, "unit": ""},
]

score1 = 0
score2 = 0
halftime = 0

pathname = os.path.dirname(__file__)
model_file = pathname + "/RF_wd_24.joblib"
rf_classifier = joblib.load(model_file)

def send_to_dashboard():
    response = requests.post("https://api.tago.io/data", headers=headers, json=payload)
    # print(response.status_code)
    # print(response.text)
    clear_payload()

def clear_payload():
    global payload
    payload = []

def add_to_payload(var, val, unit=None):
    time = datetime.now().timestamp()
    json = {'variable': str(var), 'value': val, 'timestamp': time}
    if unit is not None:
        json['unit'] = str(unit)

    # print(json)

    return json

send_to_dashboard()

# Function to calculate SMA (Signal Magnitude Area)
def calculate_sma(row):
    accel_cols = [col for col in row.index if 'A' in col]
    gyro_cols = [col for col in row.index if 'G' in col]

    accel_values = row[accel_cols]
    gyro_values = row[gyro_cols]

    sma_accel = np.mean(np.sqrt(accel_values**2)) if not accel_values.empty else 0
    sma_gyro = np.mean(np.sqrt(gyro_values**2)) if not gyro_values.empty else 0

    return sma_accel + sma_gyro

# Function to calculate IQR (Interquartile Range)
def calculate_iqr(series):
  return series.quantile(0.75) - series.quantile(0.25)

# Function to calculate Entropy
def calculate_entropy(series):
  # For accelerometer data, we can use value counts to estimate probability distribution
  value_counts = pd.cut(series, bins=10).value_counts(normalize=True)
  entropy = -np.sum(value_counts * np.log2(value_counts + 1e-9)) # Add small epsilon to avoid log(0)
  return entropy

# Function to calculate Correlation between x, y, and z
def calculate_correlation(row, axes):
  cols = [col for col in row.index if any(axis in col for axis in axes)]
  if len(cols) < 2:
      return 0
  values = row[cols]
  if values.nunique() < 2: # Need at least two unique values to compute correlation
      return 0
  return values.corr(values.shift(1)).iloc[0, 1] if values.corr(values.shift(1)).shape == (2, 2) else 0 # Using lagged correlation as a proxy

# Function to calculate Auto Regression Coefficients (simple version - first order)
def calculate_ar_coeff(series):
    if len(series) < 2 or series.std() == 0:
        return 0
    # Simple first-order autoregression coefficient
    coor = np.corrcoef(series[:-1], series[1:])[0, 1]
    if coor == 'nan':
      return -1
    else:
      return np.corrcoef(series[:-1], series[1:])[0, 1]


def increase_features(data):
  # Group columns by sensor and axis
  accel_x_cols = [col for col in data.columns if col.startswith('Ax_')]
  accel_y_cols = [col for col in data.columns if col.startswith('Ay_')]
  accel_z_cols = [col for col in data.columns if col.startswith('Az_')]

  # Calculate statistical features for each axis
  data['Ax_mean'] = data[accel_x_cols].mean(axis=1)
  data['Ax_std'] = data[accel_x_cols].std(axis=1)
  data['Ax_max'] = data[accel_x_cols].max(axis=1)
  data['Ax_min'] = data[accel_x_cols].min(axis=1)
  data['Ax_iqr'] = data[accel_x_cols].apply(calculate_iqr, axis=1)
  data['Ax_entropy'] = data[accel_x_cols].apply(calculate_entropy, axis=1)
  data['Ax_ar_coeff'] = data[accel_x_cols].apply(calculate_ar_coeff, axis=1)


  data['Ay_mean'] = data[accel_y_cols].mean(axis=1)
  data['Ay_std'] = data[accel_y_cols].std(axis=1)
  data['Ay_max'] = data[accel_y_cols].max(axis=1)
  data['Ay_min'] = data[accel_y_cols].min(axis=1)
  data['Ay_iqr'] = data[accel_y_cols].apply(calculate_iqr, axis=1)
  data['Ay_entropy'] = data[accel_y_cols].apply(calculate_entropy, axis=1)
  data['Ay_ar_coeff'] = data[accel_y_cols].apply(calculate_ar_coeff, axis=1)

  data['Az_mean'] = data[accel_z_cols].mean(axis=1)
  data['Az_std'] = data[accel_z_cols].std(axis=1)
  data['Az_max'] = data[accel_z_cols].max(axis=1)
  data['Az_min'] = data[accel_z_cols].min(axis=1)
  data['Az_iqr'] = data[accel_z_cols].apply(calculate_iqr, axis=1)
  data['Az_entropy'] = data[accel_z_cols].apply(calculate_entropy, axis=1)
  data['Az_ar_coeff'] = data[accel_z_cols].apply(calculate_ar_coeff, axis=1)


  # Calculate SMA
  data['SMA'] = data.apply(calculate_sma, axis=1)

  # do the same for Gyroscope data
  # Group columns by sensor and axis
  gyro_x_cols = [col for col in data.columns if col.startswith('Gx_')]
  gyro_y_cols = [col for col in data.columns if col.startswith('Gy_')]
  gyro_z_cols = [col for col in data.columns if col.startswith('Gz_')]

  # Calculate statistical features for each axis
  data['Gx_mean'] = data[gyro_x_cols].mean(axis=1)
  data['Gx_std'] = data[gyro_x_cols].std(axis=1)
  data['Gx_max'] = data[gyro_x_cols].max(axis=1)
  data['Gx_min'] = data[gyro_x_cols].min(axis=1)
  data['Gx_iqr'] = data[gyro_x_cols].apply(calculate_iqr, axis=1)
  data['Gx_entropy'] = data[gyro_x_cols].apply(calculate_entropy, axis=1)


  data['Gy_mean'] = data[gyro_y_cols].mean(axis=1)
  data['Gy_std'] = data[gyro_y_cols].std(axis=1)
  data['Gy_max'] = data[gyro_y_cols].max(axis=1)
  data['Gy_min'] = data[gyro_y_cols].min(axis=1)
  data['Gy_iqr'] = data[gyro_y_cols].apply(calculate_iqr, axis=1)
  data['Gy_entropy'] = data[gyro_y_cols].apply(calculate_entropy, axis=1)

  data['Gz_mean'] = data[gyro_z_cols].mean(axis=1)
  data['Gz_std'] = data[gyro_z_cols].std(axis=1)
  data['Gz_max'] = data[gyro_z_cols].max(axis=1)
  data['Gz_min'] = data[gyro_z_cols].min(axis=1)
  data['Gz_iqr'] = data[gyro_z_cols].apply(calculate_iqr, axis=1)
  data['Gz_entropy'] = data[gyro_z_cols].apply(calculate_entropy, axis=1)

  return data

def window_list_data(data, window):
  df = pd.DataFrame()
  col = 1
  while col <= window:
    df['Ax_' + str(col)] = pd.Series([float(data[col-1][1])])
    df['Ay_' + str(col)] = pd.Series([float(data[col-1][2])])
    df['Az_' + str(col)] = pd.Series([float(data[col-1][3])])
    df['Gx_' + str(col)] = pd.Series([float(data[col-1][4])])
    df['Gy_' + str(col)] = pd.Series([float(data[col-1][5])])
    df['Gz_' + str(col)] = pd.Series([float(data[col-1][6])])
    col += 1
#   print(df)
  return df

def process_data(data, window, rf_classifier):
  df = window_list_data(data, window)
  featured_df = increase_features(df)
  data = featured_df.drop(columns=featured_df.loc[:, 'Ax_1':'Gz_24'].columns)
  # Sample prediction
  # sample = X_test.iloc[0:1]  # Keep as DataFrame to match model input format
  # prediction = rf_classifier.predict(sample)
  prediction = rf_classifier.predict(data)
#   print("Prediction: ", prediction)
  return prediction

read_str = ""
ser = serial.Serial()

CSV_SIZE = 1200
data_count = 0
classif_count = 0 # count for the number of classifications found so far
htft_count = 0

def send_uart(input_str):
    global ser # check
    if (ser != None):
        try: 
            ser.write(input_str.encode('utf-8'))
        except serial.serialutil.SerialException:
            print("Unable to write to port")
    else:
        print("Port is not connected")

def reading_thread():
    try:
        global ser
        global score1
        global score2
        global payload
        global classif_count
        global htft_count

        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        count = 0
        # global data_count
        data = []
        window = 24
        shift = 5
        # add loop here where you fill data like normal

        # dirname = os.path.dirname(__file__)
        # print(dirname)
        # movement = 'scrum_adv_rand'
        # csv_name = dirname + '/data_' + movement + '.csv'
        
        # create dataframe
        df = pd.DataFrame()
        lastTime = datetime.now().timestamp()
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
                    
                    # data_count += 1
                
                if (count > 25):
                    read_str = ""
                    count = 0
                
                if (htft_count == 2):
                    # send FT score reset 
                    score1 = 0
                    score2 = 0
                    json = add_to_payload("score1", score1)
                    payload.append(json)
                    json = add_to_payload("score2", score2)
                    payload.append(json)
                    send_to_dashboard()
                    htft_count = 0

                # shift window 
                if len(data) == window:
                    prediction = process_data(data, window, rf_classifier) # this should be performed while data is still being collected
                    prediction = prediction[0].item()
                    # print(prediction)
                    time = datetime.now().timestamp()
                    # print("comparing times", lastTime, time)
                    if prediction != 0 and time-lastTime > 3:
                        if prediction == 1:
                            htft_count += 1
                        if prediction== 2:
                            if data[0][-1] == '1': # check if button is down then team 2
                                # print(data[0][-1])

                                prediction = 4
                                # print(prediction)
                                score2 += 5
                                json = add_to_payload("score2", score2)
                                payload.append(json)
                            else: # button not down team 1
                                score1 += 5
                                json = add_to_payload("score1", score1)
                                payload.append(json)
                                
                        classif_count += 1
                        uart_str = "c " + str(prediction) + " " + str(classif_count) + "\n"
                        send_uart(uart_str)

                        json = add_to_payload("Classification", prediction)
                        payload.append(json)
                        send_to_dashboard()
                        lastTime = time
                    if prediction == 0 and time-lastTime > 10:
                        json = add_to_payload("Classification", prediction)
                        payload.append(json)
                        send_to_dashboard()
                        lastTime = time - 3
                    data = data[shift:]

                # This code is for when collecting data for the machine
                # if (data_count >= CSV_SIZE):
                #     print("Reached data limit")
                #     # fill dataframe headings Ax, Ay, Az, Gz, Gy, Gz, Category
                #     df['Time'] = [data[i][0] for i in range(0, len(data))]
                #     df['Ax'] = [data[i][1] for i in range(0, len(data))]
                #     df['Ay'] = [data[i][2] for i in range(0, len(data))]
                #     df['Az'] = [data[i][3] for i in range(0, len(data))]
                #     df['Gx'] = [data[i][4] for i in range(0, len(data))]
                #     df['Gy'] = [data[i][5] for i in range(0, len(data))]
                #     df['Gz'] = [data[i][6] for i in range(0, len(data))]
                #     df['Category'] = [data[i][7] for i in range(0, len(data))]
                #     df.to_csv(csv_name, index=False)
                #     print("saved to csv " + csv_name)
                #     break

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

def send_from_user():
    write_str = tempVar.get()           # received input from user
    split_in = write_str.split(" ")

    errorLabel.config(text="")          # clear error label

    # checks shell input
    if (len(split_in) == 2):
        if (split_in[0] == "s"):
            try:
                val = int(split_in[1])
                if (val == 1 or val == 0):
                    send_uart(write_str + "\n")             # sends to uart
                else:
                    # 2nd argument is not a valid input number
                    errorLabel.config(text="Error: Invalid input value, enter 0 or 1")
            except ValueError:
                # 2nd argument is not a number
                errorLabel.config(text="Error: Invalid input type, enter an int")
        elif (split_in[0] == "c"):
            
            try:
                val = int(split_in[1])
                if (val == 1 or val == 2 or val == 3 or val == 4):
                    global classif_count
                    classif_count += 1

                    if (val == 1):
                        global htft_count
                        htft_count += 1

                    write_str = write_str + " " + str(classif_count) + "\n"
                    send_uart(write_str)                    # sends to uart
                else:
                    # 2nd argument is not a valid input number
                    errorLabel.config(text="Error: Invalid input value, enter 1~4")
            except ValueError:
                # 2nd argument is not a number
                errorLabel.config(text="Error: Invalid input type, enter an int")
        else:
            # first argument is not a valid command
            errorLabel.config(text="Error: Invalid command")
    else:
        # input has too many arguments
        errorLabel.config(text="Error: Invalid argument number")
        
    input_box.delete(0, END)                    # clears input box

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

    # dataLabel.config(text="Data count: " + str(data_count))

    main.after(100, update) # run itself again after 100 ms

varLabel = Label(top_frame, text = "Shell Commands: ")
varLabel.place(x = 50, y = 30)

tempVar = StringVar()

input_box = Entry(top_frame, textvariable = tempVar, width = 30, state = NORMAL)
input_box.place(x = 50, y = 50)
input_Button = Button(top_frame, text = "Submit", state = NORMAL, command=send_from_user)
input_Button.place(x = 50, y = 80)

shell_connect_Button = Button(top_frame, text = "Connect to shell", state = NORMAL, command=uart_read_thread)
shell_connect_Button.place(x = 450, y = 20)

# dataLabel = Label(top_frame, text = "Data count: ")
# dataLabel.place(x = 400, y = 60)

errorLabel = Label(top_frame, text = "")
errorLabel.place(x = 150, y = 80)

outputText = scrolledtext.ScrolledText(bottom_frame, height = 25, width = 60, state = NORMAL)
outputText.place(x = 50, y = 20)

update()
# Closes mainloop for window
main.mainloop()