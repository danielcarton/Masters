%-----------------Simulation parameters--------------
sampleRate = 256;     % sensor samples per second
simDuration = 4 ;   % Duration of simulation in seconds
samples = sampleRate*simDuration;   % total samples made
interSamples = 1;  % No of datapoints per sample, or datapoints in between each sample
totalSamples = sampleRate*simDuration*interSamples; % Total number of datapoints of input signal
signalFrequency = 1;    % Signal Frequency
dutyCycle = 0.5;    % Duty cycle of square-wave signal
signalAmplitude = 1;% Ampli tude of input signal


%-----------------value noise stuff------------------
valueNoiseFrequency = 4; % random data points generated per second of simulation
valueNoiseDatapoints = valueNoiseFrequency * simDuration;

%-----------------Sensor parameters------------------
resolution = 16;     % bits of resolution
maxMeasured = 1;   % sensor max value, gotta be greater than signal amplitude
minMeasured = -1;  % same^
levels = 2^resolution;  % Number of levels 
LSBvalue = (maxMeasured - minMeasured)/levels;  % LSB value of sensor
signalPower = 50;    % power of signal compared to noise in dB
maxBiasNoise = 0.1; % maximum amplitude of the bias noise
noSensors = 1;

%-----------------Creae input signals---------------
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
biasNoise = (rand(noSensors, 1)-0.5)*2*maxBiasNoise; % generate bias voltage offset within parameters from earlier
noisySine = zeros(totalSamples, noSensors);
noisySquare = zeros(totalSamples, noSensors);
noisyValueNoise = zeros(totalSamples, noSensors);
for i = 1:noSensors
    noisySine(:,i) = awgn(simpleSine, signalPower, "measured") + biasNoise(i);
    noisySquare(:,i) = awgn(simpleSquare, signalPower, "measured") + biasNoise(i);
    noisyValueNoise(:,i) = awgn(simpleValueNoise, signalPower, "measured") + biasNoise(i);
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
order = 0;
algOutSine = zeros(totalSamples/interSamples, 1);
algOutSquare = zeros(totalSamples/interSamples, 1);
algOutValueNoise = zeros(totalSamples/interSamples, 1);

for i = 1:totalSamples/interSamples
    algOutSine(i) = MultiSensorFusionSet(sensorSine, order, i);
    algOutSquare(i) = MultiSensorFusionSet(sensorSquare, order, i);
    algOutValueNoise(i) = MultiSensorFusionSet(sensorValueNoise, order, i);
end


%-----------------plots------------------------------
yaxisPadding = 1.25;

fullTimeSpace = linspace(0, simDuration, totalSamples);
reducedTimeSpace = linspace(0, simDuration, totalSamples/interSamples);

errorSine = abs(algOutSine - simpleSine(1:interSamples:totalSamples))/signalAmplitude*100;
errorSquare = abs(algOutSquare - simpleSquare(1:interSamples:totalSamples))/signalAmplitude*100;
errorValueNoise = abs(algOutValueNoise - simpleValueNoise(1:interSamples:totalSamples))/signalAmplitude*100;

meanErrorSine = mean(errorSine);
meanErrorSquare = mean(errorSquare);
meanErrorValueNoise = mean(errorValueNoise);

maxErrorSine = max(errorSine(10:size(errorSine)-10));
maxErrorSquare = max(errorSquare(10:size(errorSquare)-10));
maxErrorValueNoise = max(errorValueNoise(10:size(errorValueNoise)-10));

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
plot(reducedTimeSpace, algOutSine,'r');
title("Noisy sine Filtered");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(reducedTimeSpace, algOutSquare,'r');
title("Noisy square Filtered");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(reducedTimeSpace, algOutValueNoise,'r');
title("Noisy value noise Filtered");
ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

% Evaluation error of algorithm

nexttile;
plot(reducedTimeSpace, errorSine, 'c');
yline(meanErrorSine, '-', sprintf('Mean error: %0.2f%% of input. Max error: %0.2f%% of input', meanErrorSine, maxErrorSine));
title("Sine error");
%ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(reducedTimeSpace, errorSquare, 'c');
yline(meanErrorSquare, '-', sprintf('Mean error: %0.2f%% of input. Max error: %0.2f%% of input', meanErrorSquare, maxErrorSquare));
title("Square error");
%ylim([-1*yaxisPadding*signalAmplitude, yaxisPadding*signalAmplitude])

nexttile;
plot(reducedTimeSpace, errorValueNoise, 'c');
yline(meanErrorValueNoise, '-', sprintf('Mean error: %0.2f%% of input. Max error: %0.2f%% of input', meanErrorValueNoise, maxErrorValueNoise));
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


function estimate = MultiSensorFusionSet(sensorData, order, time)

    assert(time <= size(sensorData, 1)); % make sure sampling is within the set of data
    if (order >= time)
        order = time - 1;
    end
    
    samples = sensorData(time-order:time,1);


    assert(time <= size(sensorData, 1));    % assert we're not accessing a point beyond the timestep of the simulation TODO verify i used size correctly

    n = size(samples, 1);

    meanElementDistances = zeros(n, 1);  % This array will hold the mean distance each element has to the others, later this will be used to create the set psi
    
    for i = 1:n
        for j = 1:n
            meanElementDistances(i) = meanElementDistances(i) + abs( samples(j) - samples(i) );
        end
        meanElementDistances(i) = meanElementDistances(i)/n;
    end

    
    setMean = mean(meanElementDistances);
    %{%}
    psi = [];
    
    for i = 1:n
        if meanElementDistances(i) <= setMean
            psi = [psi samples(i)];
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
    

end
