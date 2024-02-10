%-----------------Simulation parameters--------------
sampleRate = 16;     % sensor samples per second
samplePeriod = 1/sampleRate;
simDuration = 8;   % Duration of simulation in seconds
samples = sampleRate*simDuration;   % total samples made
interSamples = 32;  % No of datapoints per sample, or datapoints in between each sample
totalSamples = sampleRate*simDuration*interSamples; % Total number of datapoints of input signal
signalFrequency = 0.5;    % Signal Frequency
dutyCycle = 0.5;    % Duty cycle of square-wave signal
signalAmplitude = 1;% Amplitude of input signal
noisePower = 25;    % power of signal compared to noise

%-----------------value noise stuff------------------
valueNoiseFrequency = 1; % random data points generated per second of simulation
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
valueNoiseRandPoints = rand(valueNoiseDatapoints + 1, 1);   % randomly generated value noise datapoints, requires additional point to complete the longer array

% Square wave and sine generation

for i = 1: totalSamples
    simpleSine(i) = signalAmplitude * sin(i / (totalSamples / simDuration) * 2 * pi * signalFrequency);

    if mod(i, totalSamples/simDuration/signalFrequency) >= dutyCycle * totalSamples/simDuration/signalFrequency
            simpleSquare(i) = signalAmplitude;
    else
            simpleSquare(i) = 0;
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

%-----------------plots------------------------------
%{%}
t = tiledlayout(3, 3);

nexttile;
plot(simpleSine);
title("Input sinusoidal signal");

nexttile;
plot(simpleSquare);
title("Input square signal");

nexttile;
plot(simpleValueNoise);
title("Input 'realistic' signal");


nexttile;
plot(noisySine);
title("Noisy sinusoidal signal");

nexttile;
plot(noisySquare);
title("Noisy square signal");

nexttile;
plot(noisyValueNoise);
title("noisy 'realistic' signal");


nexttile;
plot(sensorSine);
title("Discrete sensor reading for sinusoidal signal");

nexttile;
plot(sensorSquare);
title("Discrete sensor reading for square signal");

nexttile;
plot(sensorValueNoise);
title("Discrete sensor reading for 'realistic' signal");

t.Padding = 'compact';
t.TileSpacing = 'compact';



%-------------------Functions------------
function returnpoint = cosineInterpolate(spx, spy, epx, epy, step) % start point xy, end point xy, and step between the two
    assert(spx <= step); % Make sure step is in between start and end
    assert(step <= epx);   
    step = (step - spx)/(epx - spx);    % make step a value between 0 and 1 based on its position between start and end
    returnpoint = (1 - cos(step*pi))*0.5 * (epy - spy) + spy;
end
