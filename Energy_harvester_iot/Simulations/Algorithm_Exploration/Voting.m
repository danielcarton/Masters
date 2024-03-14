clear
% Define standard simulation parameters, should be same across all simulations
simDuration = 100;
sampleRate = 32;
samplePeriod = 1/sampleRate;
num_samples = sampleRate * simDuration;
t = linspace(0, simDuration, num_samples); % Time vector t from 0 to simDuration seconds with num_samples points
max_distance = 2000; % Maximum measurable distance by the sensor (in millimeters)
min_distance = 100; % Minimum measurable distance by the sensor (in millimeters)
change_interval = 0.2; % Interval for changing distance values (in seconds)
noise_amplitude = 36; % Amplitude of noise (in millimeters)
num_sensors = 5;

% To generate a new test signal or not, with new number of sensor this should be updated.
genSig = false;


% Generate simulated distance sensor output with reduced variability
if genSig == true
    distance = zeros(1, num_samples);
    for i = 1:num_samples
        if mod(t(i), change_interval) >= change_interval - 0.05 || i == 1
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

    save('signalNoisy.mat', 'distanceNoisy')
    save('signalClean.mat', 'noNoiseDistance')
end

if genSig == false
    signal = matfile('testSignalNoisy.mat');
    distanceNoisy = signal.distanceNoisy;

    signal = matfile('testSignalClean.mat');
    noNoiseDistance = signal.noNoiseDistance;
end


% Apply simple voting taking the average of the two closest values

distanceFiltered = zeros(1, num_samples);
votingArray = zeros(num_sensors, 1);

for i = 1:num_samples
    for j = 1:num_sensors
        votingArray(j) = distanceNoisy(j,i);
    end
    if median(votingArray) - min(votingArray) < max(votingArray) - median(votingArray)
        x = (min(votingArray) + median(votingArray))/2;
    else
        x = (max(votingArray) + median(votingArray))/2;
    end
    distanceFiltered(i) = x;
end


% Calculate absolute error between the ideal and noisy and filtered signals
for i = 1:num_sensors
    errorNoisy = abs(noNoiseDistance-distanceNoisy(i,:));
end
errorFiltered = abs(noNoiseDistance-distanceFiltered);

meanErrorNoisy = mean(errorNoisy)
meanErrorFiltered = mean(errorFiltered)

accuracy = (meanErrorNoisy/meanErrorFiltered)*100 + "%"

maxErrorNoisy = max(errorNoisy);
maxErrorFiltered = max(errorFiltered);



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

