previous_error = 0
def TEC_ctrl():
    # Start loop/thread when experiment begins
    #
    # Read temperature from reference temperature (I2C)
    # Read temperature trace (.csv generated form MATLAB?)
    # PID
    #   - Take the error between reference temperature and trace
    #   - Add error to sum_errors
    #   - Adj = KP x error + KI x previous error + KD x sum_errors
    #   - Set error to previous error
    # Output PWM signal to RPI_GPIO_22, where duty cycle is based on the adjustment from PID
    #
    # End loop/thread when experiment finishes
    
def PID(temp_error, previous_error, KP, KI, KD):
    sum_errors += temp_error
    temp_adj = KP*temp_error + KI*sum_errors + KD*previous_error
    previous_error = temp_error
    return temp_adj 
    