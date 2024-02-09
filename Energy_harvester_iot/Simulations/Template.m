%-----------------Simulation parameters--------------
sampleRate = 16;     % samples per second
simDuration = 16;   % Duration of simulation in seconds
samples = sampleRate*simDuration;   % total samples made
interSamples = 16;  % No of datapoints per sample, or datapoints in between each sample
totalSamples = sampleRate*simDuration*interSamples; % Total number of datapoints of input signal
signalPeriods = 4;  % Number of input signal periods in simulation
dutyCycle = 0.3;    % Duty cycle of square-wave signal
signalAmplitude = 1;% Amplitude of input signal
noisePower = 25;    % power of signal compared to noise

%-----------------Perlin noise stuff-----------------


%-----------------Sensor parameters------------------
resolution = 8;     % bits of resolution
maxMeasured = 10;   % sensor max value, gotta be greater than signal amplitude
minMeasured = -10;  % same^
levels = 2^resolution;  % Number of levels 
LSBvalue = (maxMeasured - minMeasured)/levels;  % LSB value of sensor

%-----------------Create input signals---------------
time = (linspace(0, simDuration, totalSamples))';

simpleSine = zeros(totalSamples, 1);    % SineWave
simpleSquare = zeros(totalSamples, 1);  % Square wave 
simplePerlin_1D = zeros(totalSamples, 1);   % Perlin

% Square wave and sine generation
i = 1;
while(i <= totalSamples)
    simpleSine(i) = signalAmplitude * sin((i/(totalSamples/signalPeriods))*2*pi);
    if mod(i, (totalSamples/signalPeriods)) >= totalSamples*dutyCycle/signalPeriods
        simpleSquare(i) = signalAmplitude;
    else
        simpleSquare(i) = 0;
    end
    i = i+1;
end


%-----------------Add white gaussian noise-----------
noisySine = awgn(simpleSine, noisePower, "measured");
noisySquare = awgn(simpleSquare, noisePower, "measured");

%-----------------Add sensor error-------------------
sensorSine = noisySine(1:interSamples:totalSamples);    % copy signals at sample rate
sensorSquare = noisySquare(1:interSamples:totalSamples);

sensorSine = round(sensorSine/LSBvalue)*LSBvalue;   % "round" to LSB value of sensor
sensorSquare = round(sensorSquare/LSBvalue)*LSBvalue;
%-----------------Output sensor values---------------
% IDK if theres anything to add here

%-----------------Algorithm time---------------------

%-----------------plots------------------------------
%{
t = tiledlayout(3, 2);

nexttile;
plot(time, simpleSine);
title("Input sinusoidal signal");

nexttile;
plot(time, simpleSquare);
title("Input square signal");

nexttile;
plot(time, noisySine);
title("Noisy sinusoidal signal");

nexttile;
plot(time, noisySquare);
title("Noisy square signal");

nexttile;
plot(sensorSine);
title("Discrete sensor reading for sinusoidal signal");

nexttile;
plot(sensorSquare);
title("Discrete sensor reading for square signal");

t.Padding = 'compact';
t.TileSpacing = 'compact';
%}

s = perlin2D(totalSamples)
mesh(s);



%-------------------Functions------------
function s = perlin2D (m)
  s = zeros([m,m]);     % Prepare output image (size: m x m)
  w = m;
  i = 0;
  while w > 3
    i = i + 1;
    d = interp2(randn([m,m]), i-1, 'spline');
    s = s + i * d(1:m, 1:m);
    w = w - ceil(w/2 - 1);
  end
  s = (s - min(min(s(:,:)))) ./ (max(max(s(:,:))) - min(min(s(:,:))));
end
