import smbus
import time
import math
import numpy as np
from numpy import binary_repr
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv

# I2C setup
bus = smbus.SMBus(1)

Sensor_addr_1 = 0x48
Sensor_addr_2 = 0x49
Sensor_addr_3 = 0x4a
Sensor_addr_4 = 0x4b

config_adr = 0x01
result_adr = 0x00
confH = 0x0
confL = 0x0

data = [confH, confL]

bus.write_i2c_block_data(Sensor_addr_1, config_adr, data)
bus.write_i2c_block_data(Sensor_addr_2, config_adr, data)
bus.write_i2c_block_data(Sensor_addr_3, config_adr, data)
bus.write_i2c_block_data(Sensor_addr_4, config_adr, data)

# Matplotlib setup for live plotting of temperature
test_duration = 6001 # 1 = 100ms, 6000 = 10min
fig, ax = plt.subplots()
xdata, ydata = [], []
line, = ax.plot([], [], lw=2)
ax.set_xlim(0, test_duration)
ax.set_ylim(0, 100)
ax.grid()

# GPIO setup
CTLI_pin = 22
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(CTLI_pin, GPIO.OUT)
pi_pwm = GPIO.PWM(CTLI_pin, 1000)
pi_pwm.start(45)  # 45 is roughly the midpoint at 1.5V

min_duty_cycle = 12
max_duty_cycle = 79

# Global variables and lists
previous_error = 0
sum_error = 0
trace_temp = 40  # Constant test trace temperature
results_sensor_1 = []
results_sensor_2 = []
results_sensor_3 = []
results_sensor_4 = []

KP = 2.5
KD = 1
KI = 0.4

with open('temperature_trace.csv', newline = '') as csv_file:
    reader = csv.reader(csv_file, delimiter = ',')
    temperature_trace = list(reader)


def PID(temp_error, previous_error, sum_error, KP, KI, KD):
    sum_error += temp_error
    temp_adj = KP * temp_error + KI * sum_error + KD * previous_error
    previous_error = temp_error
    return temp_adj, sum_error

def read_temperature(address):
    temperature = bus.read_i2c_block_data(address, result_adr, 2)
    first_int = int(binary_repr(temperature[0], 8), 2)
    second_int = int(binary_repr(temperature[1], 8), 2)
    temp = (first_int << 8 | second_int) * 0.0078125
    return temp

def maprange(a, b, s):
    (a1, a2), (b1, b2) = a, b
    return b1 + ((s - a1) * (b2 - b1) / (a2 - a1))

def init():
    ax.set_xlim(0, test_duration)
    ax.set_ylim(30, 85)
    line.set_data([], [])
    return line,

def update(frame):
    global previous_error, sum_error
    global results_sensor_1, results_sensor_2, results_sensor_3, results_sensor_4
    temp1 = read_temperature(Sensor_addr_1)
    temp2 = read_temperature(Sensor_addr_2)
    temp3 = read_temperature(Sensor_addr_3)
    temp4 = read_temperature(Sensor_addr_4)
    
    # Average all 4 sensors?
    temp_avg = (temp1 + temp2 + temp3 + temp4)/4
    
    print("{a:16.6f} {b:16.6f} {c:16.6f} {d:16.6f}". format(a = temp1, b = temp2, c = temp3, d = temp4))
    print(frame)
    temp_error = float(temperature_trace[0][frame]) - temp_avg
    #temp_error = trace_temp - temp_avg
    
    results_sensor_1.insert(frame, temp1)
    results_sensor_2.insert(frame, temp2)
    results_sensor_3.insert(frame, temp3)
    results_sensor_4.insert(frame, temp4)
    
    temp_adjustment, sum_error = PID(temp_error, previous_error, sum_error, KP, KI, KD)
    previous_error = temp_error
    pwm_adjustment = maprange((8, -8), (min_duty_cycle, max_duty_cycle), temp_adjustment)

    if pwm_adjustment > max_duty_cycle:
        pwm_adjustment = max_duty_cycle
    elif pwm_adjustment < min_duty_cycle:
        pwm_adjustment = min_duty_cycle

    pi_pwm.ChangeDutyCycle(int(pwm_adjustment))
    
    xdata.append(frame)
    ydata.append(temp_avg)
    line.set_data(xdata, ydata)
    
    if len(xdata) > test_duration:
        xdata.pop(0)
        ydata.pop(0)
        ax.set_xlim(xdata[0], xdata[-1])
    
    return line,

ani = animation.FuncAnimation(fig, update, frames=range(test_duration), init_func=init, blit=True, interval=100, repeat=False)
plt.show(block = True)

np.savetxt("reference_1_results.csv", results_sensor_1, delimiter = ",", fmt = '%s,')
np.savetxt("reference_2_results.csv", results_sensor_2, delimiter = ",", fmt = '%s,')
np.savetxt("reference_3_results.csv", results_sensor_3, delimiter = ",", fmt = '%s,')
np.savetxt("reference_4_results.csv", results_sensor_4, delimiter = ",", fmt = '%s,')

