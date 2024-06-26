%---------------Description---------------
% Generate a trace depicting some temperature over 10 minutes ranging
% from around 30 to 60 degrees. The start of the trace increases the 
% temperature until it reaches a plateau of around 60 degrees. This
% plateau continues until some time when the temperature begins to
% decrease at the end of the trace.

clear

% Parameters
duration = 10 * 60; % Duration in seconds (10 minutes)
sampling_rate = 10; % Sampling rate in Hz (1 sample per second)
time_vector = 0:1/sampling_rate:duration; % Time vector
% temperature_offset = 30;

base_temp = 30; % Base temperature
peak_temp = 60; % Peak temperature
plateau_duration = 4 * 60; % Duration of plateau in seconds

% Generate temperature trace with plateau
rise_duration = (duration - plateau_duration) / 2;
fall_duration = rise_duration;

% Create temperature trace with rise, plateau, and fall
temperature_trace = zeros(size(time_vector));

% Rising phase
rise_indices = time_vector <= rise_duration;
temperature_trace(rise_indices) = base_temp + (peak_temp - base_temp) * (time_vector(rise_indices) / rise_duration);

% Plateau phase
plateau_indices = (time_vector > rise_duration) & (time_vector <= (rise_duration + plateau_duration));
temperature_trace(plateau_indices) = peak_temp;

% Falling phase
fall_indices = time_vector > (rise_duration + plateau_duration);
temperature_trace(fall_indices) = peak_temp - (peak_temp - base_temp) * ((time_vector(fall_indices) - (rise_duration + plateau_duration)) / fall_duration);

% Add small fluctuations (can be removed)
small_fluctuations = 0.5 * cos(2 * pi * time_vector / 60) + 0.2 * sin(2 * pi * time_vector / 120);
temperature_trace = temperature_trace + small_fluctuations;

% Plot the temperature trace
figure;
plot(time_vector, temperature_trace);
title('Temperature Trace Over 10 Minutes');
xlabel('Time (seconds)');
ylabel('Temperature (°C)');
grid on;

% Optionally save the data to a file
save('realistic_temperature_trace.mat', 'time_vector', 'temperature_trace');
