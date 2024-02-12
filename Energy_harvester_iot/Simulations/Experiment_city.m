%-----------------Simulation parameters--------------
sampleRate = 32;     % sensor samples per second
samplePeriod = 1/sampleRate;
simDuration = 2 ;   % Duration of simulation in seconds
samples = sampleRate*simDuration;   % total samples made
interSamples = 8;  % No of datapoints per sample, or datapoints in between each sample
totalSamples = sampleRate*simDuration*interSamples; % Total number of datapoints of input signal
signalFrequency = 1;    % Signal Frequency
dutyCycle = 0.5;    % Duty cycle of square-wave signal
signalAmplitude = 1;% Ampli tude of input signal


%-----------------value noise stuff------------------
valueNoiseFrequency = 2; % random data points generated per second of simulation
valueNoiseDatapoints = valueNoiseFrequency * simDuration;

%-----------------Sensor parameters------------------
resolution = 8;     % bits of resolution
maxMeasured = 1;   % sensor max value, gotta be greater than signal amplitude
minMeasured = -1;  % same^
levels = 2^resolution;  % Number of levels 
LSBvalue = (maxMeasured - minMeasured)/levels;  % LSB value of sensor
noisePower = 25;    % power of signal compared to noise
maxBiasNoise = 0.1; % maximum amplitude of the bias noise
noSensors = 5;

%-----------------Create input signals---------------
simpleSine = zeros(totalSamples, 1);    % SineWave
simpleSquare = zeros(totalSamples, 1);  % Square wave 

simpleValueNoise = zeros(totalSamples, 1);   % final value noise array
valueNoiseRandPoints = (rand(valueNoiseDatapoints + 1, 1)-0.5).*2;   % randomly generated value noise datapoints, requires additional point to complete the longer array

% Square wave and sine generation

for i = 1: totalSamples
    simpleSine(i) = signalAmplitude * sin(i / (totalSamples / simDuration) * 2 * pi * signalFrequency);

    if mod(i, totalSamples/simDuration/signalFrequency) >= dutyCycle * totalSamples/simDuration/signalFrequency
            simpleSquare(i) = signalAmplitude;
    else
            simpleSquare(i) = -1*signalAmplitude;
    end 
end

clear i

% value noise generation

for i = 1:(valueNoiseDatapoints)     % Major steps, will basically increment in jumps of ValueNoiseDatapoints to cover the range, stops two early
    currentPoint = (i-1) * totalSamples/valueNoiseDatapoints ;    % Points to the place in the long array (simpleValueNoise) where the small array (valueNoiseRandPoints) datapoint would be
    nextPoint = (i) * totalSamples/valueNoiseDatapoints;   % points to the place in the long array where the next small array datapoint would be
    for j = 1: totalSamples/valueNoiseDatapoints               % minor steps, will increment within the jumps of VNDps
        spx = currentPoint + 1;
        spy = valueNoiseRandPoints(i);
        epx = nextPoint + 1;
        epy = valueNoiseRandPoints(i+1);
        step = currentPoint + j;
        simpleValueNoise(currentPoint+j) = signalAmplitude * cosineInterpolate(spx, spy, epx, epy, step);
    end
end 
clear spx spy epx epy step i j currentPoint nextPoint

%-----------------Add white gaussian noise-----------
biasNoise = (rand(noSensors) - 0.5)*2*maxBiasNoise; % generate bias voltage offset within parameters from earlier
noisySine = zeros(totalSamples, noSensors);
noisySquare = zeros(totalSamples, noSensors);
noisyValueNoise = zeros(totalSamples, noSensors);
for i = 1:noSensors
    noisySine(:,i) = awgn(simpleSine, noisePower, "measured") + biasNoise(i);
    noisySquare(:,i) = awgn(simpleSquare, noisePower, "measured") + biasNoise(i);
    noisyValueNoise(:,i) = awgn(simpleValueNoise, noisePower, "measured") + biasNoise(i);
end

%-----------------Add sensor error-------------------
sensorSine = noisySine(1:interSamples:totalSamples,:);    % copy signals at sample rate
sensorSquare = noisySquare(1:interSamples:totalSamples,:);
sensorValueNoise = noisyValueNoise(1:interSamples:totalSamples,:);

sensorSine = round(sensorSine/LSBvalue)*LSBvalue;   % "round" to LSB value of sensor
sensorSquare = round(sensorSquare/LSBvalue)*LSBvalue;
sensorValueNoise = round(sensorValueNoise/LSBvalue)*LSBvalue;
%-----------------Output sensor values---------------
% IDK if theres anything to add here

%-----------------Algorithm time---------------------
fs = 0.1;
algOutSine = zeros(totalSamples/interSamples, noSensors);
algOutSquare = zeros(totalSamples/interSamples, noSensors);
algOutValueNoise = zeros(totalSamples/interSamples, noSensors);
for i =1:noSensors
    algOutSine(:,i) = lowpass(sensorSine(:,i), fs);
    algOutSquare(:,i) = lowpass(sensorSquare(:,i), fs);
    algOutValueNoise(:,i) = lowpass(sensorValueNoise(:,i), fs);
end

%-----------------plots------------------------------
yaxisPadding = 1.25;

fullTimeSpace = linspace(0, simDuration, totalSamples);
reducedTimeSpace = linspace(0, simDuration, totalSamples/interSamples);

errorSine = zeros(totalSamples/interSamples, noSensors);
errorSquare = zeros(totalSamples/interSamples, noSensors);
errorValueNoise = zeros(totalSamples/interSamples, noSensors);

meanErrorSine = zeros(noSensors, 1);
meanErrorSquare = zeros(noSensors, 1);
meanErrorValueNoise = zeros(noSensors, 1);


for i = 1:noSensors
    errorSine(:,i) = abs(algOutSine(:,i) - simpleSine(1:interSamples:totalSamples))/signalAmplitude*100;
    errorSquare(:,i) = abs(algOutSquare(:,i) - simpleSquare(1:interSamples:totalSamples))/signalAmplitude*100;
    errorValueNoise(:,i) = abs(algOutValueNoise(:,i) - simpleValueNoise(1:interSamples:totalSamples))/signalAmplitude*100;
    
    meanErrorSine(i) = mean(errorSine(:,i));
    meanErrorSquare(i) = mean(errorSquare(:,i));
    meanErrorValueNoise(i) = mean(errorValueNoise(:,i));
end

t = tiledlayout(5, 3);

% Ideal input signals

nexttile;
plot(fullTimeSpace, simpleSine, 'b');
title("Ideal sinusoidal signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude]);

nexttile;
plot(fullTimeSpace, simpleSquare, 'b');
title("Ideal square signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
hold on;
plot(fullTimeSpace, simpleValueNoise, 'b');
hold off
title("Ideal 'realistic' signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

% Input signal with addative white noise

nexttile;
plot(fullTimeSpace, noisySine, 'r');
title("Noisy sinusoidal signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(fullTimeSpace, noisySquare, 'r');
title("Noisy square signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(fullTimeSpace, noisyValueNoise, 'r');
title("noisy 'realistic' signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

% less-sampled sensor signals

nexttile;
plot(reducedTimeSpace, sensorSine, 'g');
title("Discrete sensor reading for sinusoidal signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(reducedTimeSpace, sensorSquare, 'g');
title("Discrete sensor reading for square signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(reducedTimeSpace, sensorValueNoise, 'g');
title("Discrete sensor reading for 'realistic' signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

% Post-algorithm interpreted signal

nexttile;
plot(reducedTimeSpace, algOutSine);
title("Noisy sine Filtered");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(reducedTimeSpace, algOutSquare);
title("Noisy square Filtered");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(reducedTimeSpace, algOutValueNoise);
title("Noisy value noise Filtered");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

% Evaluation error of algorithm

nexttile;
plot(reducedTimeSpace, errorSine, 'c');
yline(meanErrorSine, '-', sprintf('Mean error: %0.2f%% of input', meanErrorSine));
title("Sine error");
%ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(reducedTimeSpace, errorSquare, 'c');
yline(meanErrorSquare, '-', sprintf('Mean error: %0.2f%% of input', meanErrorSquare));
title("Square error");
%ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(reducedTimeSpace, errorValueNoise, 'c');
yline(meanErrorValueNoise, '-', sprintf('Mean error: %0.2f%% of input', meanErrorValueNoise));
title("Value Noise error");
%ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

t.Padding = 'compact';
t.TileSpacing = 'compact';


%-------------------Functions------------
function returnpoint = cosineInterpolate(spx, spy, epx, epy, step) % start point xy, end point xy, and step between the two
    assert(spx <= step); % Make sure step is in between start and end
    assert(step <= epx);   
    step = (step - spx)/(epx - spx);    % make step a value between 0 and 1 based on its position between start and end
    returnpoint = (1 - cos(step*pi))*0.5 * (epy - spy) + spy;
end
