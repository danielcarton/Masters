profile on

% Sim Parameters
simDuration = 10;   % sim duration in seconds
totalSamples = 1000;
SignalFrequency = 0.5;
signalDataPoints = sin(2*pi*SignalFrequency*(linspace(0, simDuration, totalSamples)));  

output = zeros(1, totalSamples);
order = 5;

for i = 1:totalSamples
%    output(1, i) = RunningAverage(signalDataPoints, order, i);
save('TestBenchTemp','-append')
end

plot(signalDataPoints);
hold on
plot(output);
hold off

profileStruct = profile('info');
