clear
% Define parameters
simDuration = 5;
sampleRate = 32;
samplePeriod = 1/sampleRate;
num_samples = sampleRate * simDuration;
t = linspace(0, simDuration, num_samples); % Time vector t from 0 to simDuration seconds with num_samples points
max_distance = 2000; % Maximum measurable distance by the sensor (in millimeters)
min_distance = 100; % Minimum measurable distance by the sensor (in millimeters)
change_interval = 1; % Interval for changing distance values (in seconds)
noise_amplitude = 36; % Amplitude of noise (in millimeters)
num_sensors = 3;


% Generate simulated distance sensor output with reduced variability
distance = zeros(1, num_samples);
for i = 1:num_samples
    if mod(t(i), change_interval) >= 0.97 || i == 1
        distance(i) = min_distance + (max_distance - min_distance) * rand; % Generate new distance value
    else
        distance(i) = distance(i - 1); % Maintain previous distance value
    end
end
% Apply a simple low-pass filter (moving average)
filter_window_size = 60; % Size of the moving average window
distance_smoothed = movmean(distance, filter_window_size);


noNoiseDistance = distance_smoothed;

% Add noise to the sensor outputs
distanceNoisy = zeros(num_sensors, num_samples);
for i = 1:num_sensors
    distanceNoisy(i,:) = distance_smoothed + noise_amplitude * randn(size(distance_smoothed));
end

% Apply Kalman filtering

distanceFiltered = zeros(1, num_samples);
votingArray = zeros(num_sensors, 1);

for i = 1:num_samples
    for j = 1:num_sensors
        votingArray(j) = kalman(distanceNoisy(j,i));
    end
    if median(votingArray) - min(votingArray) < max(votingArray) - median(votingArray)
        x = (min(votingArray) + median(votingArray))/2;
    else
        x = (max(votingArray) + median(votingArray))/2;
    end
    distanceFiltered(i) = x;
end

% Low-pass filter
% distanceFiltered = lowpass(distanceFiltered, 0.1);

% Calculate absolute error between the ideal and noisy and filtered signals
for i = 1:num_sensors
    errorNoisy = abs(noNoiseDistance-distanceNoisy(i,:));
end
errorFiltered = abs(noNoiseDistance-distanceFiltered);



% Plot the simulated VL53L0X ToF ranging sensor output
figure(2);
tiledlayout(2,1);
nexttile;
plot(t, noNoiseDistance, 'g', 'LineWidth', 5);
hold on
plot(t, distanceNoisy, 'b', 'LineWidth', 2);
hold on
plot(t, distanceFiltered, 'r','LineWidth', 2);
xlabel('Time (seconds)');
ylabel('Distance (millimeters)');
title('Simulated VL53L0X ToF Ranging Sensor Output Kalman Filtered');
legend('Ideal distance measurement', 'Distance with noise', 'Kalman filtered distance')
grid on;
hold off

nexttile;
plot(t, errorNoisy, 'b', 'LineWidth', 2);
hold on
plot(t, errorFiltered, 'r', 'LineWidth', 2);
xlabel('Time (seconds)');
ylabel('Distance error (millimeters)');
legend('Noise error', 'Kalman filtered error')
grid on
hold off


sum(errorFiltered-errorNoisy)/num_samples

save('Distance_sensorTemp','-append')
save('Distance_sensorTempTemp','-append')
function returnPoint = kalman(z)
    persistent A H Q R 
    persistent x P
    persistent firstRun
    
    if isempty(firstRun)
      A = 1;
      H = 1;
      
      Q = 9;
      R = 45;
      % Initial guesses
      x = 200;
      P = 100;
      
      firstRun = 1;  
    end
    
    % Kalman algorithm  
    % Prediction step
    xp = A*x;           
    Pp = A*P*A' + Q;   
    
    % Kalman gain
    K = Pp*H'*inv(H*Pp*H' + R); 
    
    % Estimation step
    x = xp + K*(z - H*xp); 
    P = Pp - K*H*Pp;     
    
    returnPoint = x;
save('Distance_sensorTempTemp','-append')
end

% save('Distance_sensorTemp','-append')

