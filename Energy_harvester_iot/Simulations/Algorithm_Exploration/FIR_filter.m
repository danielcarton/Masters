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

% To generate a new test signal or not. If new number of sensor this should be updated
genSig = false;


% Generate simulated generic sensor output with reduced variability
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
title('Simulated sensor output using optimal set algorithm');
legend('Ideal sensor output', 'Output with noise', 'Optimal set')
grid on;
hold off

nexttile;
plot(t, errorNoisy, 'b', 'LineWidth', 2);
hold on
plot(t, errorFiltered, 'r', 'LineWidth', 2);
% yline(meanErrorNoisy, '-', sprintf('Mean error: %0.2f% of input. Max error: %0.2f% of input', meanErrorNoisy, maxErrorNoisy));
% yline(meanErrorFiltered, '-', sprintf('Mean error: %0.2f% of input. Max error: %0.2f% of input', meanErrorFiltered, maxErrorFiltered));
% xlabel('Time (seconds)');
ylabel('Sensor error');
legend('Noise error', 'Optimal set error')
grid on
hold off


sum(errorFiltered-errorNoisy)/num_samples

function estimate = FIRfilter(Sensordata, TimeStep)

% Ts = 2kHZ, fcutoff = 500Hz, Lowpass
% filter = [-0.02010411882885732, -0.05842798004352509, -0.061178403647821976, -0.010939393385338943, 0.05125096443534972, 0.033220867678947885, -0.05655276971833928, -0.08565500737264514, 0.0633795996605449, 0.31085440365663597, 0.4344309124179415, 0.31085440365663597, 0.0633795996605449, -0.08565500737264514, -0.05655276971833928, 0.033220867678947885, 0.05125096443534972, -0.010939393385338943, -0.061178403647821976, -0.05842798004352509, -0.02010411882885732];

% Ts = 2kHZ, fcutoff = 100Hz, Lowpass
%filter = [-0.005946901269245116, -0.003221105934752317, -0.003844069990276836, -0.004335185893314174, -0.004618082667223955, -0.004610544806595842, -0.004229573289430783, -0.003391745396869743, -0.0020280510416206413, -0.0000779737869300066, 0.002492691628631225, 0.005699827658770489, 0.009534812572960348, 0.013961833701309632, 0.018917781140524174, 0.02430952185560112, 0.030016610101736976, 0.03589781173597167, 0.04180040186066689, 0.04755534497978652, 0.052984157628638756, 0.057920150927138224, 0.062213967331716837, 0.06570516590449127, 0.06829411291109368, 0.069881349050518, 0.07041715978547827, 0.069881349050518, 0.06829411291109368, 0.06570516590449127, 0.062213967331716837, 0.057920150927138224, 0.052984157628638756, 0.04755534497978652, 0.04180040186066689, 0.03589781173597167, 0.030016610101736976, 0.02430952185560112, 0.018917781140524174, 0.013961833701309632, 0.009534812572960348, 0.005699827658770489, 0.002492691628631225, -0.0000779737869300066, -0.0020280510416206413, -0.003391745396869743, -0.004229573289430783, -0.004610544806595842, -0.004618082667223955, -0.004335185893314174, -0.003844069990276836, -0.003221105934752317, -0.005946901269245116];

% Ts = 1kHZ, fcutoff = 100Hz, Lowpass
% filter = [-0.007775715121256268, -0.007938974136595613, -0.009534265788246128, -0.008779578259641298, -0.004381884421750879, 0.004666131585205163, 0.0188044731228937, 0.03764144706001848, 0.05992101812383003, 0.08357444021744635, 0.10601855702701225, 0.12454015906119098, 0.13674393462068657, 0.14100385434561774, 0.13674393462068657, 0.12454015906119098, 0.10601855702701225, 0.08357444021744635, 0.05992101812383003, 0.03764144706001848, 0.0188044731228937, 0.004666131585205163, -0.004381884421750879, -0.008779578259641298, -0.009534265788246128, -0.007938974136595613, -0.007775715121256268];

% Ts = 100Hz, fcutoff = 20Hz, Lowpass
 filter = [-0.01259277478717816, -0.02704833486706803, -0.031157016036431583, -0.003351666747179282, 0.06651710329324828, 0.1635643048779222, 0.249729473226146, 0.2842779082622769, 0.249729473226146, 0.1635643048779222, 0.06651710329324828, -0.003351666747179282, -0.031157016036431583, -0.02704833486706803, -0.01259277478717816];

n = size(filter, 1)

for i = TimeStep - n: TimeStep
    if (i <= 0)
        continue;
    end
    index = i + n - TimeStep;
    estimate = estimate + Sensordata(1, i) * filter(1, index);
end

end
