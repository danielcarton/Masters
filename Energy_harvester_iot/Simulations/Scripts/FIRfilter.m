function estimate = FIRfilter(Sensordata, TimeStep)

filter = [];    % replace with calculated values
n = size(filter, 1)

for i = TimeStep - n: TimeStep
    if (i <= 0)
        continue;
    end
    index = i + n - TimeStep;
    estimate = estimate + Sensordata(1, i) * filter(1, index);
end

end