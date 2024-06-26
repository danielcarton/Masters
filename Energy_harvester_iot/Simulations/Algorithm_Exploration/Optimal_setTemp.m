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
num_sensors = 6;

% To generate a new test signal or not, with new number of sensor this should be updated
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



% Apply optimal fusion set algorithm
for i = 1:num_samples
    distanceFiltered = MultisensorFusionSet(distanceNoisy, i);
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


% Plot the simulated sensor output
figure(2);
tiledlayout(2,1);
nexttile;
plot(t, noNoiseDistance, 'g', 'LineWidth', 5);
hold on
plot(t, distanceNoisy, 'b', 'LineWidth', 2);
hold on
plot(t, distanceFiltered, 'r','LineWidth', 2);
xlabel('Time (seconds)');
ylabel('Sensor output');
title('Simulated sensor output with ');
legend('Ideal distance measurement', 'Distance with noise', 'Kalman filtered distance')
grid on;
hold off

nexttile;
plot(t, errorNoisy, 'b', 'LineWidth', 2);
hold on
plot(t, errorFiltered, 'r', 'LineWidth', 2);
% yline(meanErrorNoisy, '-', sprintf('Mean error: %0.2f% of input. Max error: %0.2f% of input', meanErrorNoisy, maxErrorNoisy));
% yline(meanErrorFiltered, '-', sprintf('Mean error: %0.2f% of input. Max error: %0.2f% of input', meanErrorFiltered, maxErrorFiltered));
% xlabel('Time (seconds)');
ylabel('Distance error (millimeters)');
legend('Noise error', 'Kalman filtered error')
grid on
hold off


sum(errorFiltered-errorNoisy)/num_samples
save('Optimal_setTemp','-append')

function estimate = MultisensorFusionSet(sensorData, time)
    assert(time <= size(sensorData, 2));    % assert we're not accessing a point beyond the timestep of the simulation TODO verify i used size correctly

    n = size(sensorData, 1);

    meanElementDistances = zeros(n, 1);  % This array will hold the mean distance each element has to the others, later this will be used to create the set psi
    
    for i = 1:n
        for j = 1:n
            meanElementDistances(i) = meanElementDistances(i) + abs( sensorData(j, time) - sensorData(i, time) );
        end
        meanElementDistances(i) = meanElementDistances(i)/n;
    end

    
    setMean = mean(meanElementDistances);
    %{%}
    psi = [];
    
    for i = 1:n
        if meanElementDistances(i) <= setMean
            psi = [psi sensorData(i, time)];
        end
    end
    

   % psi = sensorData(time, :);
    m = size(psi, 2);

    C = zeros(m);
    for i = 1:m
        for j = 1:m
            C(i, j) = exp( (-1/2) * abs( psi(i) - psi(j) ) );
        end
    end


    mu = zeros(1, m);

    for i = 1:m
        for j = 1:m
            mu(1, i) = mu(1, i) + C(i, j);
        end
        mu(1, i) = mu(1, i)/m;
    end

    tau = zeros (1, m);
    
    for i = 1:m
        sum = 0;
        for j = 1:m
            sum = sum + ( mu(1, i) - C(i, j) )^2;
        end
        sum = sum/m;
        tau(1, i) = 1/sum;
    end

    omega = tau + mu;

    w = zeros(1, m);
    S = 0;
    for i = 1:m
        S = S + omega(1, i);
    end

    for i = 1:m
        w(1, i) = omega(1, i)/S;
    end
    
    estimate = 0;
    for i = 1:m
        estimate = estimate + w(1, i) * psi(1, i);
    end
    

save('Optimal_setTemp','-append')
end

