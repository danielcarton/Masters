
from multiprocessing import Process, Event, Value, Array, Manager
import threading

from modules.Peripherals import Peri, ADC, DAC, LED, System_States
from modules.Auxilary import Emulator, Bluetooth

import RPi.GPIO as GPIO
from smbus2 import SMBus, i2c_msg
import modules.pinOut_BCM as pinOut
from modules.file_handler import file
import parameters as pm

import pandas as pd
import csv
import time
from datetime import datetime
import socket

# Create events
exit_event = Event()
end_voltage_reached_event = Event()
start_voltage_reached_event = Event()
env_dataset_done = Event()


# Create shared variables
shared_V_th = Value('d', 0.0)   # DC/DC turn-on-threshold
shared_V_hyst = Value('d', 0.0)     # DC/DC turn-off-threshold = V_th - V_hyst
Dynamic_Banks = Value('i',0)    # Dynamic or static energy storage banks
performance_log = Array('i', [0, 0]) # Sampling/Processing, Communication


Peri()
dac_instance = DAC()

# GPIO.setup(pinOut.UART_TXD, GPIO.OUT)
# GPIO.setup(pinOut.UART_RXD, GPIO.IN)

bus = SMBus(1)

def main():

    print("Starting main")
    GPIO.output(pinOut.UART_TXD, GPIO.LOW)
    time.sleep(1)

    # Define if static or dynamic energy storage banks
    Dynamic_Banks.value = 1

    # Define Irradiance Dataset
    #dataset = "winter_28days_start_cut.tsv"
    #dataset = "autumn_17days_start_cut.tsv"
    #dataset = "summer_31days_start_cut.tsv"
    #dataset = "3_winter_3_autumn_2_summer_start_cut.tsv"
    #dataset = "Test_500W.tsv"
    dataset = "Test_700W-300W_Mix_Long.tsv"
    #dataset = "Test_200W-700W-changes.tsv"

    # Define temperature trace datasets
    PLATEAU_TEST_TRACE = 'plateau_trace.csv'
    SINE_TEST_TRACE = 'sine_trace.csv'
    SQUARE_TEST_TRACE = 'square_trace.csv'
    STAIR_TEST_TRACE = 'stair_trace.csv'
    ARCH_TEST_TRACE = 'arch_trace.csv'
    TRIANGLE_TEST_TRACE = 'triangle_trace.csv'
    CONSTANT_TEST_TRACE = 'constant_trace.csv'

    if Dynamic_Banks.value:
        print("Dynamic Banks ACTIVATED!")
    else:
        print("Dynamic Banks DEACTIVATED!")
    time.sleep(2)

    sys_states = init_sys_states()

    manager = Manager()
    data_log, adc_channels = init_data_log(manager)


    system_states_instance = System_States(sys_states)
    adc_instance = ADC(system_states_instance, end_voltage_reached_event, start_voltage_reached_event, shared_V_th, shared_V_hyst, Dynamic_Banks)
    #emulator_instance = Emulator(dac_instance)
    #led_instance = LED(data_log)
    #bluetooth_instance = Bluetooth()

    # Attaching Data_Ready Callback
    GPIO.add_event_detect(pinOut.ADC1_DRDY, GPIO.FALLING, callback=adc_instance.data_ready_callback)

    time.sleep(1)

     

    # Run experiements with different temperature traces
    # Exp_Temp_Evaluation(adc_instance, adc_channels, sys_states, dataset, data_log, bus, PLATEAU_TEST_TRACE, disable_data_ready = False, KP = 0.4, KD = 0.2, KI = 0.08)
    # time.sleep(3)

    # Exp_Temp_Evaluation(adc_instance, adc_channels, sys_states, dataset, data_log, bus, SINE_TEST_TRACE, disable_data_ready = False, KP = 0.4, KD = 0.2, KI = 0.08)
    # time.sleep(3)

    Exp_Temp_Evaluation(adc_instance, adc_channels, sys_states, dataset, data_log, bus, SQUARE_TEST_TRACE, disable_data_ready = False, KP = 0.4, KD = 0.2, KI = 0.08)
    time.sleep(3)

    Exp_Temp_Evaluation(adc_instance, adc_channels, sys_states, dataset, data_log, bus, STAIR_TEST_TRACE, disable_data_ready = False, KP = 0.4, KD = 0.2, KI = 0.08)
    time.sleep(3)

    Exp_Temp_Evaluation(adc_instance, adc_channels, sys_states, dataset, data_log, bus, ARCH_TEST_TRACE, disable_data_ready = False, KP = 0.4, KD = 0.2, KI = 0.08)
    time.sleep(3)

    Exp_Temp_Evaluation(adc_instance, adc_channels, sys_states, dataset, data_log, bus, TRIANGLE_TEST_TRACE, disable_data_ready = True, KP = 0.4, KD = 0.2, KI = 0.08)
    time.sleep(3)

    # Constant temperature test
    # Exp_Temp_Evaluation(adc_instance, adc_channels, sys_states, dataset, data_log, bus, CONSTANT_TEST_TRACE, disable_data_ready = True, KP = 4, KD = 1.5, KI = 0.4)
    # time.sleep(3)

    print("All experiments finished!")




def init_sys_states():
    # sys_states array: (B1, B2, B3, B4, B5, MPPT, DCDC)
    # Components will be deactivated in System_States init
    sys_states = Array('i', (1, 1, 1, 1, 1, 1, 1))
    return sys_states


def init_data_log(manager):

    data_log = manager.dict()

    timeStamp = "datetime"
    irrValue = "irrValue"
    ledPercent = "ledPercent"

    btConnect = "btConnectedFlag"
    # btDisconnect    = "btDisconnectFlag"
    btPackets = "btPackets"

    adc1ch0 = "EH_IN_+_BUF"
    adc1ch1 = "CSA_STORAGE_IN_+"
    adc1ch2 = "V_BANK1"
    adc1ch3 = "V_BANK2"
    adc1ch4 = "V_BANK3"
    adc1ch5 = "V_BANK4"
    adc1ch6 = "V_BANK5"
    adc1ch7 = "STORAGE_OUT"

    adc2ch0 = "DCDC_OUT_BUF"
    adc2ch1 = "EXT_AN_IN_1"
    adc2ch2 = "EXT_AN_IN_2"
    adc2ch3 = "EXT_CSA"
    adc2ch4 = "CSA_EH_IN"
    adc2ch5 = "CSA_STORAGE_IN"
    adc2ch6 = "CSA_STORAGE_OUT"
    adc2ch7 = "CSA_DCDC_OUT"

    MPPT_EN = "MPPT_EN"
    DCDC_EN = "DCDC_EN"
    BANK1_EN = "BANK1_EN"
    BANK2_EN = "BANK2_EN"
    BANK3_EN = "BANK3_EN"
    BANK4_EN = "BANK4_EN"
    BANK5_EN = "BANK5_EN"

    adc_channels = [adc1ch0, adc1ch1, adc1ch2, adc1ch3, adc1ch4, adc1ch5, adc1ch6, adc1ch7, adc2ch0, adc2ch1, adc2ch2, adc2ch3, adc2ch4, adc2ch5, adc2ch6, adc2ch7]

    data_log.update({timeStamp: 0,
                irrValue: 0,
                ledPercent: 0,
                btConnect: 0,
                # btDisconnect:   0,
                btPackets: 0,

                adc1ch0: 0,
                adc1ch1: 0,
                adc1ch2: 0,
                adc1ch3: 0,
                adc1ch4: 0,
                adc1ch5: 0,
                adc1ch6: 0,
                adc1ch7: 0,

                adc2ch0: 0,
                adc2ch1: 0,
                adc2ch2: 0,
                adc2ch3: 0,
                adc2ch4: 0,
                adc2ch5: 0,
                adc2ch6: 0,
                adc2ch7: 0,
                MPPT_EN: 0,
                DCDC_EN: 0,
                BANK1_EN: 1,
                BANK2_EN: 0,
                BANK3_EN: 0,
                BANK4_EN: 0,
                BANK5_EN: 0,
                })

    return data_log, adc_channels


def drain_energy_storage():
    print("Starting Energy Drain")
    start_voltage_reached_event.clear()
    dac_instance.current_sink(100)
    while not start_voltage_reached_event.is_set():
        time.sleep(0.1)
    dac_instance.current_sink(0)
    print("Draining Stage 1 finished")
    time.sleep(3)
    start_voltage_reached_event.clear()

    dac_instance.current_sink(2)
    while not start_voltage_reached_event.is_set():
        time.sleep(0.1)
    dac_instance.current_sink(0)
    print("Draining Stage 2 finished")
    time.sleep(1)
    start_voltage_reached_event.clear()
    
    
def PID(temp_error, previous_error, sum_error, KP, KI, KD):
        sum_error += temp_error
        temp_adj = KP*temp_error + KI*sum_error + KD*previous_error
        previous_error = temp_error
        return temp_adj, sum_error

def read_temperature(address, bus):
    # Result register address
    result_adr = 0x00
    
    # Read result register and convert result into degrees Celsius
    temp = bus.read_i2c_block_data(address, result_adr, 2)
    first_bin = bin(temp[0])
    second_bin = bin(temp[1])

    first_bin_split = first_bin.split('b')
    second_bin_split = second_bin.split('b')

    first_int = int(first_bin_split[1], 2)
    second_int = int(second_bin_split[1], 2)

    temperature = first_int << 8 | second_int
    temperature = temperature*0.0078125
    return temperature

# Function to map one range to another
def maprange(a, b, s):
    (a1, a2), (b1, b2) = a, b
    return b1 + ((s - a1) * (b2 - b1) / (a2 - a1))

def PID_loop(bus, CURRENT_TEST_TRACE, KP, KD, KI):
    
    # Device addresses
    Sensor_addr_1 = 0x48
    Sensor_addr_2 = 0x49
    Sensor_addr_3 = 0x4a
    Sensor_addr_4 = 0x4b

    # Internal device registers
    config_adr = 0x01
    result_adr = 0x00

    # Set configuration register
    confH = 0x00
    confL = 0x00
    data = [confH, confL]

    # Write config to I2C devices
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
    trace_temp = 40 # Constant trace temperature for testing
    results_sensor_1 = []
    results_sensor_2 = []
    results_sensor_3 = []
    results_sensor_4 = []



    

    with open(CURRENT_TEST_TRACE, newline = '') as csv_file:
        reader = csv.reader(csv_file, delimiter = ',')
        temperature_trace = list(reader)
    run_iteration = 0
    seconds_per_sample = 0.1
    while(run_iteration < 6001):
        # Store start time
        start_time = time.time()
        
        # Read reference sensor temperatures
        temp1 = read_temperature(Sensor_addr_1, bus)
        temp2 = read_temperature(Sensor_addr_2, bus)
        temp3 = read_temperature(Sensor_addr_3, bus)
        temp4 = read_temperature(Sensor_addr_4, bus)
        
        # Average all 4 sensors used to control the TEC element
        temp_avg = (temp1 + temp2 + temp3 + temp4)/4
            
        print("{a:40.6f} {b:16.6f} {c:16.6f} {d:16.6f}". format(a = temp1, b = temp2, c = temp3, d = temp4))

        print(CURRENT_TEST_TRACE, ": ", run_iteration)

        # Calculate the error between trace and average
        temp_error = float(temperature_trace[0][run_iteration]) - temp_avg
        
        results_sensor_1.insert(run_iteration, temp1)
        results_sensor_2.insert(run_iteration, temp2)
        results_sensor_3.insert(run_iteration, temp3)
        results_sensor_4.insert(run_iteration, temp4)
            
        # Use PID controller to determine temperature adjustment
        temp_adjustment, sum_error = PID(temp_error, previous_error, sum_error, KP, KI, KD)
        previous_error = temp_error
        # Map and convert the temperature adjusment to duty cycle
        pwm_adjustment = maprange((8, -8), (min_duty_cycle, max_duty_cycle), temp_adjustment)

        if pwm_adjustment > max_duty_cycle:
            pwm_adjustment = max_duty_cycle
        elif pwm_adjustment < min_duty_cycle:
            pwm_adjustment = min_duty_cycle

        # Output new duty cycle
        pi_pwm.ChangeDutyCycle(int(pwm_adjustment))
        
        # Update run iteration and wait the remaining time until 0.1 seconds
        run_iteration = run_iteration + 1
        time.sleep(seconds_per_sample - (time.time()-start_time))

    pi_pwm.ChangeDutyCycle(45)

    rows = zip(results_sensor_1, results_sensor_2, results_sensor_3, results_sensor_4, temperature_trace[0])
    with open(f"{CURRENT_TEST_TRACE[:len(CURRENT_TEST_TRACE)-4]}_reference_results.csv", "w") as f:
        writer = csv.writer(f)
        for row in rows:
            writer.writerow(row)


def Exp_Temp_Evaluation(adc_instance, adc_channels, sys_states, dataset, data_log, bus, CURRENT_TEST_TRACE, disable_data_ready, KP, KD, KI):

    print("Starting test: ", CURRENT_TEST_TRACE)
    

     # OPEN NEW FILE
    filename = f"{CURRENT_TEST_TRACE[:len(CURRENT_TEST_TRACE)-4]}_ADC_results.csv"  # Create a unique filename
    file_instance = file(dataset, filename)
    file_instance.create_output_file(data_log)  # Create the output file
    print("Output File Created")
    adc_instance.filename.value = filename.encode('utf-8')

    print("Telling SoC a test has been started")   
    GPIO.output(pinOut.UART_TXD, GPIO.HIGH)
        
            
    # Start Processes
    processes = []
    p_Proc_A = Process(target=adc_instance.Data_Processing_A, args=(exit_event, dataset, data_log, adc_channels, sys_states))
    p_Proc_B = Process(target=adc_instance.Data_Processing_B, args=(exit_event, dataset, data_log, adc_channels, sys_states))

    processes.append(p_Proc_A)
    processes.append(p_Proc_B)

    p_Proc_A.start()
    p_Proc_B.start()


    PID_loop(bus, CURRENT_TEST_TRACE, KP, KD, KI)



    print("Finishing Experiment, telling SoC test is finished")

    GPIO.output(pinOut.UART_TXD, GPIO.LOW)

    time.sleep(2)

    exit_event.set()  # Set the event to signal the child process to exit
    # completing process
    for p in processes:
        p.join()
    exit_event.clear()

    # Disabling Data_Ready Callback
    if disable_data_ready == True:
        GPIO.remove_event_detect(pinOut.ADC1_DRDY)
    time.sleep(1)



    # Waiting for SoC to signal that data is available in storage
    while(GPIO.input(pinOut.UART_RXD) == 0):
        print("Nothing received from SoC...")
        time.sleep(0.5)


    print("Now retrieving from storage")
    deviceaddress = 0x50
    writeaddress = 0x00
    addresses= [0, 0, 0]
    # break_flag = 0
    # consecutivezeros = 0
    
    with open(f"/home/pi/Desktop/{CURRENT_TEST_TRACE[:len(CURRENT_TEST_TRACE)-4]}_output.bin", "wb") as myfile:
        for i in range(4096):
            # if break_flag == 1:
            #     break
            addresses[0] = deviceaddress | (writeaddress >> 16) & 0x07
            addresses[1] = (writeaddress >> 8) & 0xff
            addresses[2] = writeaddress & 0xff

            msg = i2c_msg.write(addresses[0], [addresses[1], addresses  [2]])
            bus.i2c_rdwr(msg)
            msg = i2c_msg.read(addresses[0], 128)
            bus.i2c_rdwr(msg)
            #print(msg.buf)
            writeaddress = writeaddress + 128

            # 2: i2c_msg is iterable
            for value in msg:
                # if value == 0:
                #     consecutivezeros += 1
                # else:
                #     consecutivezeros = 0
                # if consecutivezeros == 30:
                #     break_flag = 1
                #     break
                myfile.write(value.to_bytes(1, 'big'))







if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        exit_event.set()  # Set the event to signal the child process to exit
        # completing process
        #led_instance.set_brightness(0)
        time.sleep(1)
        GPIO.cleanup()
        print("Exiting now.")
