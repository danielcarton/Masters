function estimate = MultisensorFusionSet(sensorData, time)
    assert(time <= size(sensorData, 1));    % assert we're not accessing a point beyond the timestep of the simulation TODO verify i used size correctly

    n = size(sensorData, 2);

    meanElementDistances = zeros(n, 1);  % This array will hold the mean distance each element has to the others, later this will be used to create the set psi
    
    for i = 1:n
        for j = 1:n
            meanElementDistances(i) = meanElementDistances(i) + abs( sensorData(time, j) - sensorData(time, i) );
        end
        meanElementDistances(i) = meanElementDistances(i)/n;
    end

    
    setMean = mean(meanElementDistances);
    %{%}
    psi = [];
    
    for i = 1:n
        if meanElementDistances(i) <= setMean
            psi = [psi sensorData(time, i)];
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