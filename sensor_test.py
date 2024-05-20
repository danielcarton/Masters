import smbus
import time
import math
import numpy as np
import RPi.GPIO as GPIO
import csv

bus = smbus.SMBus(1)

Sensor_addr_1 = 0x48
Sensor_addr_2 = 0x49
Sensor_addr_3 = 0x4a
Sensor_addr_4 = 0x4b


config_adr = 0x01
result_adr = 0x00
confH = 0x00
confL = 0x00

data = [confH, confL]

bus.write_i2c_block_data(Sensor_addr_1, config_adr, data)
bus.write_i2c_block_data(Sensor_addr_2, config_adr, data)
bus.write_i2c_block_data(Sensor_addr_3, config_adr, data)
bus.write_i2c_block_data(Sensor_addr_4, config_adr, data)

CTLI_pin = 22
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(CTLI_pin, GPIO.OUT)
pi_pwm = GPIO.PWM(CTLI_pin, 1000)
pi_pwm.start(45) # 45 is roughly the midpoint at 1.5V

min_duty_cycle = 12
max_duty_cycle = 79

previous_error = 0
sum_error = 0
trace_temp = 40 # Constant test trace temperature
results_sensor_1 = []
results_sensor_2 = []
results_sensor_3 = []
results_sensor_4 = []
#run_iteraton = 0

KP = 0.4 #1
KD = 0.2 #0.5
KI = 0.08 #0.1

#CURRENT_TEST_TRACE = 'plateau_trace.csv'
#CURRENT_TEST_TRACE = 'sine_trace.csv'
#CURRENT_TEST_TRACE = 'square_trace.csv'
CURRENT_TEST_TRACE = 'stair_trace.csv'
#CURRENT_TEST_TRACE = 'arch_trace.csv'
#CURRENT_TEST_TRACE = 'triangle_trace.csv'

with open(CURRENT_TEST_TRACE, newline = '') as csv_file:
    reader = csv.reader(csv_file, delimiter = ',')
    temperature_trace = list(reader)


def PID(temp_error, previous_error, sum_error, KP, KI, KD):
    sum_error += temp_error
    temp_adj = KP*temp_error + KI*sum_error + KD*previous_error
    previous_error = temp_error
    return temp_adj, sum_error

def read_temperature(address):
     config_temp = bus.read_i2c_block_data(address, config_adr, 2)
     #print(config_temp)
     config_reg1 = config_temp[0]
     
     temperature = bus.read_i2c_block_data(address, result_adr, 2)
     first_bin = bin(temperature[0])
     second_bin = bin(temperature[1])

     first_bin_split = first_bin.split('b')
     second_bin_split = second_bin.split('b')

     first_int = int(first_bin_split[1], 2)
     second_int = int(second_bin_split[1], 2)

     temp = first_int << 8 | second_int
     temp = temp*0.0078125
     return temp
     
def maprange(a, b, s):
     (a1, a2), (b1, b2) = a, b
     return b1 + ((s - a1) * (b2 - b1) / (a2 - a1))

def PID_loop():
     run_iteraton = 0
     global previous_error, sum_error
     while(run_iteraton < 6001):
            
          temp1 = read_temperature(Sensor_addr_1)
          temp2 = read_temperature(Sensor_addr_2)
          temp3 = read_temperature(Sensor_addr_3)
          temp4 = read_temperature(Sensor_addr_4)
            
          # Average all 4 sensors
          temp_avg = (temp1 + temp2 + temp3 + temp4)/4
            
          print("{a:16.6f} {b:16.6f} {c:16.6f} {d:16.6f}". format(a = temp1, b = temp2, c = temp3, d = temp4))
          print(run_iteraton)
          temp_error = float(temperature_trace[0][run_iteraton]) - temp_avg
          #temp_error = trace_temp - temp_avg
            
          results_sensor_1.insert(run_iteraton, temp1)
          results_sensor_2.insert(run_iteraton, temp2)
          results_sensor_3.insert(run_iteraton, temp3)
          results_sensor_4.insert(run_iteraton, temp4)
            
          temp_adjustment, sum_error = PID(temp_error, previous_error, sum_error, KP, KI, KD)
          previous_error = temp_error
          pwm_adjustment = maprange((8, -8), (min_duty_cycle, max_duty_cycle), temp_adjustment)

          if pwm_adjustment > max_duty_cycle:
               pwm_adjustment = max_duty_cycle
          elif pwm_adjustment < min_duty_cycle:
               pwm_adjustment = min_duty_cycle

          pi_pwm.ChangeDutyCycle(int(pwm_adjustment))
            
          run_iteraton = run_iteraton + 1
          time.sleep(0.1)
            
PID_loop()

rows = zip(results_sensor_1, results_sensor_2, results_sensor_3, results_sensor_4, temperature_trace[0])
with open('reference_results.csv', "w") as f:
     writer = csv.writer(f)
     for row in rows:
          writer.writerow(row)

# np.savetxt("reference_1_results.csv", results_sensor_1, delimiter = ",", fmt = '%s,')
# np.savetxt("reference_2_results.csv", results_sensor_2, delimiter = ",", fmt = '%s,')
# np.savetxt("reference_3_results.csv", results_sensor_3, delimiter = ",", fmt = '%s,')
# np.savetxt("reference_4_results.csv", results_sensor_4, delimiter = ",", fmt = '%s,')

