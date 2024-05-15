clear
simDuration = 10; % Total duration of sim
samplesPerSec = 50; % Samples per second of sim
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
        signal(1, index) = 25*cosineInterpolate(spx, spy, epx, epy, index);
    end
end
clear spx spy epx epy index signalDataPoints bigStep smallStep signalpointsPerSec


% algorithm time\

output = nan(4, totalSamples);
threshold = 0.5;
activeSamples = zeros(1, 2);
index = 1;
m = 0; 
b = 0;
upperbound = 0;
lowerbound = 0;

boundaries = zeros(3, totalSamples);

while(index <= totalSamples)
    active = signal(index);
    if(index ~= 1)
        upperbound = m * index + b + threshold*sqrt(m^2+1);
        centre = m*index+b;
        lowerbound = m * index + b - threshold* sqrt(m^2+1);
        boundaries(1, index) = upperbound;
        boundaries(2, index) = lowerbound;
        boundaries(3, index) = centre;
    end
    % test if a sample is within bounds: Continue to next i if yes
    if(signal(index) > lowerbound && signal(index) < upperbound)
        index = index + 1;
        continue;
    end

    % sample two consecutive points according to sample rate
    if(index > totalSamples-1)
        break;
    end

    activeSamples = [signal(index) signal(index +1)];

    % calculate linear function of the two consecutive points
    m = (activeSamples(1, 2) - activeSamples(1, 1));
    b = activeSamples(1, 1) - m * (index-1);
    output(1, index:index+1) = activeSamples;


    % change boundaryFactor to later test if a sample is within bounds
    index = index + 2;
end
clear threshold index m b upperbound lowerbound activeSamples active cenre
output([2 3],:) = [output(1, :); output(1, :)];

% Interpret between samples
for lower = 1:totalSamples
    % find a lower boundary
    if(isnan(output(1, lower)))
        continue
    end

    for upper = lower+1:totalSamples
        % find an upper boundary
        if(isnan(output(1, upper)))
            continue
        end

        % If it reaches this spot, the upper and lower boundaries are found
        % and are not adjacent
        for step = lower:upper
            output(2, step) = cosineInterpolate(lower, output(1, lower), upper, output(1, upper), step);
            output(3, step) = linearInterpolate(lower, output(1, lower), upper, output(1, upper), step);
        end
        lower = upper;
    end
end

output(4,:) = spline(1:totalSamples, output(1,:), 1:totalSamples);

for i = 1:totalSamples
    if(isnan(output(2, i)))
        output(2, i) = output(2, i-1);
    end
    if(isnan(output(3, i)))
        output(3, i) = output(3, i-1);
    end
end

clear step upper lower i

output(1, isnan(output(1, :))) = 0;


% Plots


tiledlayout(3, 1);
nexttile
plot(t, signal);
hold on
area(t, output(1, :));
plot(t, boundaries(1, :));
plot(t, boundaries(2, :));
plot(t, boundaries(3, :));
legend('signal', 'sample', 'upper', 'lower', 'centre');

hold off
title("Signal & Sampling")
nexttile
plot(t, output(3,:));
hold on
plot(t, output(2,:));
plot(t, output(4,:));
title("interpolation")
legend('Linear Interpolation', 'Cosine Interpolation', 'Spline Interpolation)')
hold off
nexttile
plot(t, abs(signal-output(2, :)))
hold on
plot(t, abs(signal-output(3, :)))
plot(t, abs(signal-output(4, :)))
hold off
legend("Cosine interpolate Error", "Linear interpolation Error", 'Spline Interpolation Error');
title("Interpolate error")  


fprintf("Samples: %d, ignored samples: %d, efficiency: %.02f%%, mean cosine error = %.02fmC, mean linear error = %.02fmC, mean spline error = %.02fmC\n", totalSamples, nnz(~output(1, :)), nnz(~output(1, :))/totalSamples*100, mean(abs(signal-output(2, :)))*1000, mean(abs(signal-output(3, :)))*1000, mean(abs(signal-output(4, :)))*1000)


% Functions

function returnpoint = cosineInterpolate(spx, spy, epx, epy, step) % start point xy, end point xy, and step between the two
    step = (step - spx)/(epx - spx);    % make step a value between 0 and 1 based on its position between start and end
    returnpoint = (1 - cos(step*pi))*0.5 * (epy - spy) + spy;
end

function returnpoint = linearInterpolate(spx, spy, epx, epy, step)
     step = (step - spx)/(epx - spx);    % make step a value between 0 and 1 based on its position between start and end    
     returnpoint = step*(epy-spy) + spy;
end