clear
simDuration = 10; % Total duration of sim
samplesPerSec = 100; % Samples per second of sim
totalSamples = simDuration * samplesPerSec;
signalpointsPerSec = 0.5;
signal = zeros(1, totalSamples);


signalDataPoints = rand(signalpointsPerSec*simDuration + 1, 1);
t = linspace(0, simDuration, totalSamples);
for bigStep = 1:signalpointsPerSec*simDuration
    for smallStep = 1:samplesPerSec/signalpointsPerSec
        spx = (bigStep - 1) * samplesPerSec/signalpointsPerSec;
        spy = signalDataPoints(bigStep);
        epx = bigStep * samplesPerSec/signalpointsPerSec;
        epy = signalDataPoints(bigStep + 1);

        index = spx + smallStep;
        signal(1, index) = cosineInterpolate(spx, spy, epx, epy, index);
    end
end
clear spx spy epx epy index signalDataPoints bigStep smallStep


% algorithm time - Really simple, remove this when trying yourself

confidence = 0;
maxSampleRate = 1;
minSampleRate = 100;
index = 1;
prev = 0;
curr = 0;
delta = 0;
output = NaN(1, totalSamples);

while (index <= totalSamples)
    % sample
    curr = signal(index);

    % find difference in two samples
    delta = abs(curr-prev);

    % change confidence
    if(delta < 0.00003 || ~confidence)
        confidence = 1;
    else
        confidence = 0;
    end

    % add to plottable array
    output(index) = curr;

    % move index based on confidence
    index = round(index + (minSampleRate-maxSampleRate) * confidence + maxSampleRate);

    % change current and previous
    prev = curr;

end
output(isnan(output)) = 0;


tiledlayout(2, 1);
nexttile
plot(t, signal);
nexttile
plot(t, output);

% End algorithm

function returnpoint = cosineInterpolate(spx, spy, epx, epy, step) % start point xy, end point xy, and step between the two
    step = (step - spx)/(epx - spx);    % make step a value between 0 and 1 based on its position between start and end
    returnpoint = (1 - cos(step*pi))*0.5 * (epy - spy) + spy;
end