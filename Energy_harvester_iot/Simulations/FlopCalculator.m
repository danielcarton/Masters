flopsWithoutScript = 2002;    % First FLOP pass with no algorithm selected
flopsWithScript = 100;      % second FLOP pass with algorithm implemented
totalSamples = 10;          % How many samples, or in this case how many times the algorithm is done in the sim

flopsPerUse = (flopsWithScript-flopsWithoutScript)/totalSamples;