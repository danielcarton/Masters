% Running average filter for single sensor use
% How to use: 
%   - on your array of sensordata (1xN) use the RunningAverage function to
%   get a running average at a specific timeStep and a average order
%   - The output of the function will be the calculated result, given as a
%   1x1 double
%   - If the order is greater than the timestep, the order will be reduced
%   to calculate a running average as if though the order was equal to the
%   timestep
% Inputs:
%   - SensorData: 1xN double array with data from sensors
%   - Order: Order of Average filter, or how many elements are included in
%   the calculation per timestep
%   - TimeStep: The time point within the SensorData at which the estimate
%   is to be calculated from. The range of collected data is from
%   Sensordata(1, TimeStep - Order) to SensorData(1, TimeStep) given both
%   those points are within the array SensorData.
%   

function estimate = RunningAverage(SensorData, Order, TimeStep)
    estimate = 0;
    newOrder = Order;
    for i = TimeStep-Order: TimeStep
        if(i <= 0)
            newOrder = TimeStep;
            continue;
        end
    
        estimate = estimate + SensorData(1, i);
    end

    estimate = estimate/newOrder;


end