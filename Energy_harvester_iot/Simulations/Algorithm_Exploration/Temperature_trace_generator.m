clear
%-----------------Simulation parameters--------------
sampleRate = 64;     % sensor samples per second
samplePeriod = 1/sampleRate;
simDuration = 600 ;   % Duration of simulation in seconds
samples = sampleRate*simDuration;   % total samples made
interSamples = 32;  % No of datapoints per sample, or datapoints in between each sample
totalSamples = sampleRate*simDuration*interSamples; % Total number of datapoints of input signal
signalAmplitude = 20;% Amplitude of input signal
signalOffset = 40;

%-----------------value noise stuff------------------
valueNoiseFrequency = 2; % random data points generated per second of simulation
valueNoiseDatapoints = valueNoiseFrequency * simDuration;


%-----------------Create input signals---------------
simpleValueNoise = zeros(totalSamples, 1);   % final value noise array
valueNoiseRandPoints = (rand(valueNoiseDatapoints + 1, 1)-0.5).*2;   % randomly generated value noise datapoints, requires additional point to complete the longer array


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
        simpleValueNoise(currentPoint+j) = signalAmplitude * cosineInterpolate(spx, spy, epx, epy, step) + signalOffset;
    end
end 
clear spx spy epx epy step i j currentPoint nextPoint


% Ideal input signals
yaxisPadding = 1.25;

fullTimeSpace = linspace(0, simDuration, totalSamples);

nexttile;
plot(fullTimeSpace, simpleValueNoise, 'b');
title("Ideal 'realistic' signal");

function returnpoint = cosineInterpolate(spx, spy, epx, epy, step) % start point xy, end point xy, and step between the two
    assert(spx <= step); % Make sure step is in between start and end
    assert(step <= epx);   
    step = (step - spx)/(epx - spx);    % make step a value between 0 and 1 based on its position between start and end
    returnpoint = (1 - cos(step*pi))*0.5 * (epy - spy) + spy;
end