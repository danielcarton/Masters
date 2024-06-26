#include <zephyr/kernel.h>
#include <math.h>

#include "accuracyAlgorithms.h"

// #define DEBUG

// Optimal set algorithm for improving accuracy of the measurements, 
// data is a pointer to the array of data
// noSensors is the number of sensors used, number between 1 and 6
// noSamples is the number of samples from each samples, but for this only the first elements are taken
float optimalSet(float *data, uint8_t noSensors){
    uint16_t n = noSensors, m = 0;
    float meanElementDistances[MAX_SENSORS], setMean = 0;
    float psi[MAX_SENSORS], C[MAX_SENSORS][MAX_SENSORS], mu[MAX_SENSORS], tau[MAX_SENSORS], omega[MAX_SENSORS], w[MAX_SENSORS];
    float estimate;

    // TODO FIX DIVISIONS BY 0
    
    #ifdef DEBUG
        printk("Optimal Set DEBUG:\n\r Input data: ");
        for (int i = 0; i < noSensors; i++)
        {
            printk("%.3f ", data[i]);
        }
        printk("\n\r");
        
    #endif // DEBUG

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

    #ifdef DEBUG
        printk("Distances: ");
        for (int i = 0; i < noSensors; i++)
        {
            printk("%.3f ", meanElementDistances[i]);
        }
        printk("\n\r");
        
    #endif // DEBUG

    // Find the mean of the mean distances, or the set's mean distances
    for (int i = 0; i < n; i++)
    {
        setMean += meanElementDistances[i];
    }
    setMean = setMean/n;

    #ifdef DEBUG
        printk("Setmean: %.3f\n\r", setMean);
    #endif // DEBUG

    // For every item in meanElementDistances, add it to the array psi if it's mean distance to every other datapoint is LESS than the mean of means, or the set's mean (setMean)
    uint8_t psiIndex = 0;
    for (int i = 0; i < n; i++)
    {
        if (meanElementDistances[i] < setMean){ 
            psi[psiIndex] = *(data + i);
            psiIndex++;
        }
    }
    m = psiIndex;

    #ifdef DEBUG
        printk("Psi: ");
        for (int i = 0; i < psiIndex; i++)
        {
            printk("%.3f ", psi[i]);
        }
        printk("\n\r");
        
    #endif // DEBUG

    // Populate the 6x6 array C with the "fusion degree" of two different sensor values from psi. The fusion degree is an exponential value which maps the difference between any two data points in psi on an exponential curve between 1 and 0. Due to the nature of this algorithm, C will likely never be fully populated
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < m; j++)
        {
            C[i][j] = exp( (-1.0/2.0) * fabs(psi[i] - psi[j]));
        }
    }

    #ifdef DEBUG
        printk("C: \n\r");
        for (int i = 0; i < psiIndex; i++)
        {
            for (int j = 0; j < psiIndex; j++)
            {
                printk("%.3f ", C[i][j]);
            }
            printk("\n\r");
        }
        
    #endif // DEBUG
    
    // Populate the array mu with the average fusion degree of one sensor from the optimal set. This gives a value of "how fused" the sensor's measurement is compared to the others
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < m; j++)
        {
            mu[i] += C[i][j];
        }
        mu[i] = mu[i]/m;        
    }

    #ifdef DEBUG
        printk("Mu: ");
        for (int i = 0; i < psiIndex; i++)
        {
            printk("%.3f ", mu[i]);
        }
        printk("\n\r");
        
    #endif // DEBUG

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

    #ifdef DEBUG
        printk("Tau: ");
        for (int i = 0; i < psiIndex; i++)
        {
            printk("%.3f ", tau[i]);
        }
        printk("\n\r");
        
    #endif // DEBUG
    
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

    #ifdef DEBUG
        printk("Weights: ");
        for (int i = 0; i < psiIndex; i++)
        {
            printk("%.3f ", w[i]);
        }
        printk("\n\r");
        
    #endif // DEBUG
    
    // Finally, an estimate is calculated by multiplying each optimal set sensor in psi with their weight coefficients found in w
    estimate = 0;
    for (int i = 0; i < m; i++)
    {
        estimate += w[i]*psi[i];
    }

    #ifdef DEBUG
        printk("Estimate: %f\n\n\r", estimate);
    #endif // DEBUG

    return estimate;
}


// Simple average calculation of all samples and all sensors. noSensors and noSamples MUST be witin the bounds of the data array to avoid segmentation faults.
// data is a pointer to the array of data. !! This array is always 6xN !!
// noSensors is the number of sensors used, number between 1 and 6
// noSamples is the number of samples from each samples
float average(float *data, uint8_t noSensors){
    if(noSensors > 6  || noSensors < 1){
        return 0;
    }

    float sum = 0;
    for (int i = 0; i < noSensors; i++)
    {
        sum += data[i];
    }
    
    return sum / noSensors;
}

// Take average of two closest values
float simpleVoting(float *data, uint8_t noSensors){
    float estimate = -256.0;
    float minDist = 1000.0, dist;
    for (int i = 0; i < noSensors-1; i++)
    {
        for (int j = i+1; j < noSensors; j++)
        {
            dist = fabs(data[i]-data[j]);
            if (dist < minDist){
                estimate = (data[i]+data[j])/2;
                minDist = dist;
            }
        }
    }
    return estimate;
}


float x = 30.0;
float P = 10.0;

float kalman(float *data, uint8_t noSenors){
    // kalman coefficients 
    float A = 1.0; 
    float H = 1.0; 
    float Q = 9.0; 
    float R = 45.0;

    // Prediction step
    float xp = A * x;
    float Pp = (A * A * P) + Q;
    
    // Kalman gain
    float K = Pp * H / ( (H * H * Pp) + R); 
    
    // Estimation step
    x = xp + K * (data[0] - (H * xp) ); 
    P = Pp - K * H * Pp;
    
    return (x);
}