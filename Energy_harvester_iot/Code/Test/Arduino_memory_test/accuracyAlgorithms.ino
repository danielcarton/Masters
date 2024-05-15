#include "accuracyAlgorithms.h"

// Optimal set algorithm for improving accuracy of the measurements, 
// data is a pointer to the array of data
// noSensors is the number of sensors used, number between 1 and 6
// noSamples is the number of samples from each samples, but for this only the first elements are taken
float optimalSet(float *data, u8 noSensors, u16 noSamples){
    u16 n = noSensors, m = 0;
    float meanElementDistances[MAX_SENSORS], setMean = 0;
    float psi[MAX_SENSORS], C[MAX_SENSORS][MAX_SENSORS], mu[MAX_SENSORS], tau[MAX_SENSORS], omega[MAX_SENSORS], w[MAX_SENSORS];
    float estimate;

    // TODO FIX DIVISIONS BY 0

    // For each of the sensors' datapoints, calculate the mean distance between THAT sensor and every other's
    for (int i = 0; i < n; i++)
    {
        meanElementDistances[i] = 0;
        for (int j = 0; j < n; j++)
        {
            meanElementDistances[i] += fabs(*(data + j) - *(data + i));
        }
        meanElementDistances[i] = meanElementDistances[i]/n;
    }

    // Find the mean of the mean distances, or the set's mean distances
    for (int i = 0; i < n; i++)
    {
        setMean += meanElementDistances[i];
    }
    setMean = setMean/n;

    // For every item in meanElementDistances, add it to the array psi if it's mean distance to every other datapoint is LESS than the mean of means, or the set's mean (setMean)
    u8 psiIndex = 0;
    for (int i = 0; i < n; i++)
    {
        if (meanElementDistances[i] < setMean){     // TODO double-check if its < or <=
            psi[psiIndex] = *(data + i);
            psiIndex++;
        }
    }
    m = psiIndex;

    // Populate the 6x6 array C with the "fusion degree" of two different sensor values from psi. The fusion degree is an exponential value which maps the difference between any two data points in psi on an exponential curve between 1 and 0. Due to the nature of this algorithm, C will likely never be fully populated
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < m; j++)
        {
            C[i][j] = exp( (-1/2) * fabs(psi[i] - psi[j]));
        }
    }
    
    // Populate the array mu with the average fusion degree of one sensor from the optimal set. This gives a value of "how fused" the sensor's measurement is compared to the others
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < m; j++)
        {
            mu[i] += C[i][j];
        }
        mu[i] = mu[i]/m;        
    }

    // Populate the array tau with the distribution equilibrium degree of the sensor. the distribution equilibrium degree is a value showing how "stable" the fusion degree is relative to the sensors it compares to. This is a calculation similar to the standard deviation of the set, here showing a sensor's fusion degree standard deviation. This is raised to the power of -1 to make it a value betwee 0 and 1
    float sum;
    for (int i = 0; i < m; i++)
    {
        sum = 0;
        for (int j = 0; j < m; j++)
        {
            sum += pow(mu[i] - C[i][j], 2);
        }
        sum = sum/m;
        tau[i] = 1/sum;
        
    }
    
    // The array omega is the sum of mu and tau, giving the weight coefficient of each sensor, and how how weight the sensor's reading has on the final value
    for (int i = 0; i < m; i++)
    {
        omega[i] = tau[i] + mu[i];
    }
    
    // the values in omega are normalized by dividing each element in the array omega by the sum of the array omega. This normalized value is put into an array w
    sum = 0;
    for (int i = 0; i < m; i++)
    {
        sum += omega[i];
    }
    for (int i = 0; i < m; i++)
    {
        w[i] = omega[i]/sum;
    }
    
    // Finally, an estimate is calculated by multiplying each optimal set sensor in psi with their weight coefficients found in w
    estimate = 0;
    for (int i = 0; i < m; i++)
    {
        estimate += w[i]*psi[i];
    }

    return estimate;
}


// Simple average calculation of all samples and all sensors. noSensors and noSamples MUST be witin the bounds of the data array to avoid segmentation faults.
// data is a pointer to the array of data. !! This array is always 6xN !!
// noSensors is the number of sensors used, number between 1 and 6
// noSamples is the number of samples from each samples
float average(float *data, u8 noSensors, u16 noSamples){
    if(noSensors > 6  || noSensors < 1){
        return 0;
    }

    float sum = 0;
    for (int i = 0; i < noSensors; i++)
    {
        for (int j = 0; j < noSamples; j++)
        {
            sum += *(data + i + j * MAX_SENSORS);
        }
    }
    
    return sum / (noSensors * noSamples);
}