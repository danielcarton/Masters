%-----------------Simulation parameters--------------
sampleRate = 32;     % sensor samples per second
samplePeriod = 1/sampleRate;
simDuration = 5;   % Duration of simulation in seconds
samples = sampleRate*simDuration;   % total samples made
interSamples = 16;  % No of datapoints per sample, or datapoints in between each sample
totalSamples = sampleRate*simDuration*interSamples; % Total number of datapoints of input signal
signalFrequency = 1;    % Signal Frequency
dutyCycle = 0.5;    % Duty cycle of square-wave signal
signalAmplitude = 1;% Ampli tude of input signal
reducedTimeSpace = linspace(0, simDuration, totalSamples/interSamples);


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
maxBiasNoise = 1; % maximum amplitude of the bias noise
noSensors = 1;

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

% ideal temperature measurements

startTemp = 25;

tempArray = -0.01 + (0.01+0.01)*rand(samples,1);
realTemp = zeros(samples,1) + startTemp;
realTemp = realTemp + tempArray;

%idealTemp = [0; 10; 20; 30; 40; 50; 60; 70]

%-----------------Add white gaussian noise-----------
biasNoise = (rand(noSensors) - 0.5)*2*maxBiasNoise; % generate bias voltage offset within parameters from earlier
noisySine = zeros(totalSamples, noSensors);
noisySquare = zeros(totalSamples, noSensors);
noisyValueNoise = zeros(totalSamples, noSensors);
noisyTemp = zeros(samples, noSensors);
for i = 1:noSensors
    noisySine(:,i) = awgn(simpleSine, noisePower, "measured") + biasNoise(i);
    noisySquare(:,i) = awgn(simpleSquare, noisePower, "measured") + biasNoise(i);
    noisyValueNoise(:,i) = awgn(simpleValueNoise, noisePower, "measured") + biasNoise(i);
    noisyTemp = awgn(realTemp, noisePower, "measured") + biasNoise(i);
end


%-----------------Add sensor error-------------------
sensorSine = noisySine(1:interSamples:totalSamples,:);    % copy signals at sample rate
sensorSquare = noisySquare(1:interSamples:totalSamples,:);
sensorValueNoise = noisyValueNoise(1:interSamples:totalSamples,:);
sensorTemp = noisyTemp;

meanError = 0;

sensorTemp = sensorTemp + meanError;

sensorSine = round(sensorSine/LSBvalue)*LSBvalue;   % "round" to LSB value of sensor
sensorSquare = round(sensorSquare/LSBvalue)*LSBvalue;
sensorValueNoise = round(sensorValueNoise/LSBvalue)*LSBvalue;
%-----------------Output sensor values---------------
% IDK if theres anything to add here

%-----------------Algorithm time---------------------


tempTime = linspace(0, simDuration, samples);

alphaReg = alpha(tempTime,sensorTemp,samples);
betaReg = beta(tempTime,sensorTemp, samples, alpha(tempTime,sensorTemp,samples));

linReg = @(x) alphaReg*x + betaReg:


averageNoisyTemp = sum(noisyTemp)/samples

averageRealTemp = sum(realTemp)/samples

tempNoisyError = abs(averageRealTemp - averageNoisyTemp)

tempSensorError = abs(averageRealTemp - linReg(simDuration/2))


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


errorSine = zeros(totalSamples/interSamples, noSensors);
errorSquare = zeros(totalSamples/interSamples, noSensors);
errorValueNoise = zeros(totalSamples/interSamples, noSensors);

meanErrorSine = zeros(noSensors, 1);
meanErrorSquare = zeros(noSensors, 1);
meanErrorValueNoise = zeros(noSensors, 1);


% Not in use
meanErrorTempPre = sum(abs(realTemp-noisyTemp))/samples;

meanErrorTemp = sum(abs(realTemp-noisyTemp))/samples;

meanTemp = sum(realTemp)/samples;

meanErrorTempPercent = (meanErrorTemp / meanTemp) * 100;
% 

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

% nexttile;
% hold on;
% plot(fullTimeSpace, simpleValueNoise, 'b');
% hold off
% title("Ideal 'realistic' signal");
% ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
hold on;
plot(tempTime, realTemp, 'b');
hold off
title("Ideal temp signal");

% Input signal with addative white noise

nexttile;
plot(fullTimeSpace, noisySine, 'r');
title("Noisy sinusoidal signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(fullTimeSpace, noisySquare, 'r');
title("Noisy square signal");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

% nexttile;
% plot(fullTimeSpace, noisyValueNoise, 'r');
% title("noisy 'realistic' signal");
% ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(tempTime, noisyTemp, 'r');
title("noisy temp signal");

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
% % plot(reducedTimeSpace, sensorValueNoise, 'g');
% title("Discrete sensor reading for 'realistic' signal");
% ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

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
fplot(linReg, [0 simDuration])
%plot(reducedTimeSpace, algOutValueNoise);
title("Noisy temp linear regression");


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

% nexttile;
% plot(reducedTimeSpace, errorValueNoise, 'c');
% yline(meanErrorValueNoise, '-', sprintf('Mean error: %0.2f%% of input', meanErrorValueNoise));
% title("Value Noise error");
% %ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(tempTime, abs(realTemp-noisyTemp), 'c');
yline(meanErrorTemp, '-', sprintf('Mean error: %0.2f% of input', meanErrorTemp));
title("Temp error");
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


function returnPoint = alpha(reducedTimeSpace,sensorValue,samples)
    xi = 0;
    xi2 = 0;
    yi = 0;
    xiyi = 0;

    for i = 1:samples
        xi = xi + reducedTimeSpace(i);
        xi2 = xi2 + reducedTimeSpace(i)^2;
        yi = yi + sensorValue(i);
        xiyi = xiyi + reducedTimeSpace(i)*sensorValue(i);
    end
    returnPoint = (xiyi - xi*yi)/(xi2-xi^2);
end

function returnPoint = beta(reducedTimeSpace,sensorValue,samples, alpha)
    xi = 0;
    yi = 0;

    for i = 1:samples
        xi = xi + reducedTimeSpace(i);
        yi = yi + sensorValue(i);
    end
    returnPoint = (1/samples)*yi - alpha*(1/samples)*xi;
end
