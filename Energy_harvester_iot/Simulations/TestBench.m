% Sim Parameters
startFreq = 1;
endFreq = 100000;
interDecadeSamples = 10;
totalSamplesFinal = (log10(endFreq) - log10(startFreq)) * interDecadeSamples; 
freqMultiplier = nthroot(10, interDecadeSamples);
noPeriods = 10;
PeriodResoultion = 100000;
currentFreq = startFreq;
samples = PeriodResoultion*noPeriods;
freqs = 1:totalSamplesFinal;

for i = 1:totalSamplesFinal

% make signal with current frequency and given number of periods
a = linspace(0, noPeriods, samples)*currentFreq;
a = sin(a);

% do algorithm to singal a


freq = fft(a);
freqs(i) = freq(round(currentFreq));
currentFreq = currentFreq * freqMultiplier;
end

loglog(freqs)