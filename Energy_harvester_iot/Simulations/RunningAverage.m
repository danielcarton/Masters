%-----------------Simulation parameters--------------
sampleRate = 64;     % sensor samples per second
samplePeriod = 1/sampleRate;
simDuration = 8 ;   % Duration of simulation in seconds
samples = sampleRate*simDuration;   % total samples made
interSamples = 32;  % No of datapoints per sample, or datapoints in between each sample
totalSamples = sampleRate*simDuration*interSamples; % Total number of datapoints of input signal
signalFrequency = 1;    % Signal Frequency
dutyCycle = 0.5;    % Duty cycle of square-wave signal
signalAmplitude = 1;% Amplitude of input signal
noisePower = 25;    % power of signal compared to noise

%-----------------value noise stuff------------------
valueNoiseFrequency = 2; % random data points generated per second of simulation
valueNoiseDatapoints = valueNoiseFrequency * simDuration;

%-----------------Sensor parameters------------------
resolution = 8;     % bits of resolution
maxMeasured = 1;   % sensor max value, gotta be greater than signal amplitude
minMeasured = -1;  % same^
levels = 2^resolution;  % Number of levels 
LSBvalue = (maxMeasured - minMeasured)/levels;  % LSB value of sensor

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
noisySine = awgn(simpleSine, noisePower, "measured");
noisySquare = awgn(simpleSquare, noisePower, "measured");
noisyValueNoise = awgn(simpleValueNoise, noisePower, "measured");

%-----------------Add sensor error-------------------
sensorSine = noisySine(1:interSamples:totalSamples);    % copy signals at sample rate
sensorSquare = noisySquare(1:interSamples:totalSamples);
sensorValueNoise = noisyValueNoise(1:interSamples:totalSamples);

sensorSine = round(sensorSine/LSBvalue)*LSBvalue;   % "round" to LSB value of sensor
sensorSquare = round(sensorSquare/LSBvalue)*LSBvalue;
sensorValueNoise = round(sensorValueNoise/LSBvalue)*LSBvalue;
%-----------------Output sensor values---------------
% IDK if theres anything to add here

%-----------------Algorithm time---------------------
order = 3;

algOutSine = runningAverage(sensorSine, order);
algOutSquare = runningAverage(sensorSquare, order);
algOutValueNoisese = runningAverage(sensorValueNoise, order);

%-----------------plots------------------------------
yaxisPadding = 1.25;

fullTimeSpace = linspace(0, simDuration, totalSamples);
reducedTimeSpace = linspace(0, simDuration, totalSamples/interSamples);

errorSine = abs(algOutSine - simpleSine(1:interSamples:totalSamples))/signalAmplitude*100;
errorSquare = abs(algOutSquare - simpleSquare(1:interSamples:totalSamples))/signalAmplitude*100;
errorValueNoise = abs(algOutValueNoisese - simpleValueNoise(1:interSamples:totalSamples))/signalAmplitude*100;

meanErrorSine = mean(errorSine);
meanErrorSquare = mean(errorSquare);
meanErrorValueNoise = mean(errorValueNoise);

t = tiledlayout(5, 3);

% Ideal input signals

nexttile;
plot(fullTimeSpace, simpleSine);
title("Ideal sinusoidal signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude]);

nexttile;
plot(fullTimeSpace, simpleSquare);
title("Ideal square signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
hold on;
plot(fullTimeSpace, simpleValueNoise);
hold off
title("Ideal 'realistic' signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

% Input signal with addative white noise

nexttile;
plot(fullTimeSpace, noisySine);
title("Noisy sinusoidal signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(fullTimeSpace, noisySquare);
title("Noisy square signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(fullTimeSpace, noisyValueNoise);
title("noisy 'realistic' signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

% less-sampled sensor signals

nexttile;
plot(reducedTimeSpace, sensorSine);
title("Discrete sensor reading for sinusoidal signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(reducedTimeSpace, sensorSquare);
title("Discrete sensor reading for square signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(reducedTimeSpace, sensorValueNoise);
title("Discrete sensor reading for 'realistic' signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

% Post-algorithm interpreted signal

nexttile;
plot(reducedTimeSpace, algOutSine);
title("Noisy sine Lowpass Filtered");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(reducedTimeSpace, algOutSquare);
title("Noisy square Lowpass Filtered");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(reducedTimeSpace, algOutValueNoisese);
title("Noisy value noise Lowpass Filtered");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

% Evaluation error of algorithm

nexttile;
plot(reducedTimeSpace, errorSine);
yline(meanErrorSine, '-', sprintf('Mean error: %0.2f%% of input', meanErrorSine));
title("Sine error");
%ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(reducedTimeSpace, errorSquare);
yline(meanErrorSquare, '-', sprintf('Mean error: %0.2f%% of input', meanErrorSquare));
title("Square error");
%ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(reducedTimeSpace, errorValueNoise);
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

function returnArray = runningAverage(inputArray, order)
    returnArray = inputArray;
    for i = 1:size(inputArray)
        temptotal = 0;
        for j = (i - order + 1):i
            if(j < 1)
                continue;
            end
            temptotal = temptotal + inputArray(j);
        end
        returnArray(i) = temptotal/order;
    end
end
