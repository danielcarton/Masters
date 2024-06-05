#include <zephyr/kernel.h>
#include <math.h>

#include "timers.h"
#include "TempSensor.h"
#include "accuracyAlgorithms.h"
#include "memory.h"

#define DEBUG 

struct k_timer adaptive_sampling_timer;

float thresh_b = 0, thresh_m = 0, thresh_lastMeasure = 0, boundFactor = 0;
int16_t thresh_index = 256;

float bollinger_bbmid = 0, psi = 2; 

K_WORK_DEFINE(threshold_work, threshold_work_handler);
K_WORK_DEFINE(bollinger_work, bollinger_work_handler);
K_WORK_DEFINE(linearDis_work, linearDis_work_handler);
K_WORK_DEFINE(onlyAccAlg_work, onlyAccAlg_work_handler);

void timer_init_threshold_ad(void){
    k_timer_stop(&adaptive_sampling_timer);
    k_timer_init(&adaptive_sampling_timer, threshold_ad, NULL);
    k_timer_start(&adaptive_sampling_timer, K_MSEC(SAMPLE_RATE_MS), K_FOREVER);
}

void timer_init_bollinger_ad(void){
    k_timer_stop(&adaptive_sampling_timer);
    k_timer_init(&adaptive_sampling_timer, bollinger_ad, NULL);
    k_timer_start(&adaptive_sampling_timer, K_MSEC(SAMPLE_RATE_MS), K_FOREVER);
}

void timer_init_linearDis_ad(void){
    k_timer_stop(&adaptive_sampling_timer);
    k_timer_init(&adaptive_sampling_timer, linearDis_ad, NULL);
    k_timer_start(&adaptive_sampling_timer, K_MSEC(SAMPLE_RATE_MS), K_FOREVER);
}

void timer_init_onlyAccAlg(void){
    k_timer_stop(&adaptive_sampling_timer);
    k_timer_init(&adaptive_sampling_timer, onlyAccAlg, NULL);
    k_timer_start(&adaptive_sampling_timer, K_MSEC(SAMPLE_RATE_MS), K_MSEC(SAMPLE_RATE_MS));
}

void timer_stop_ad(void){
    k_timer_stop(&adaptive_sampling_timer);
}

extern void threshold_ad(struct k_timer *timer){
    k_work_submit(&threshold_work);
}

extern void bollinger_ad(struct k_timer *timer){
    k_work_submit(&bollinger_work);
}

extern void linearDis_ad(struct k_timer *timer){
    k_work_submit(&linearDis_work);
}

extern void onlyAccAlg(struct k_timer *timer){
    k_work_submit(&onlyAccAlg_work);
}

extern void threshold_work_handler(struct k_work *work)
{
    thresh_index++; 
    uint8_t memoryData[6];


    float sample, measurement;
    float samples_for_measurement[DYNAMIC_NOSENSORS];
    temp_Measure_Read(&sample, AVG_1, 1);

    float predicted = (thresh_index * thresh_m) + thresh_b+1;
    #ifdef DEBUG
        printk("Predicted value: %.3f\n\r", predicted);
    #endif // DEBUG

    if (thresh_index == 1){
        #ifdef DEBUG
            printk("Thresh index 1\n\r");
        #endif // DEBUG


        temp_Measure_Read(samples_for_measurement, AVG_1, DYNAMIC_NOSENSORS);

        measurement = optimalSet(samples_for_measurement, DYNAMIC_NOSENSORS);
        // measurement = average(samples_for_measurement, DYNAMIC_NOSENSORS);
        // measurement = simpleVoting(samples_for_measurement, DYNAMIC_NOSENSORS);
        #ifdef DEBUG
            printk("Measurement: %.3f\n\r", measurement);
        #endif // DEBUG

        thresh_m = measurement - thresh_lastMeasure;
        thresh_b = thresh_lastMeasure;
        boundFactor = THRESHOLD_SIZE * sqrt((thresh_m * thresh_m) + 1.0);
        #ifdef DEBUG
            printk("m: %.3f, b: %.3f, Boundary factor: %.3f\n\r", thresh_m, thresh_b, boundFactor);
        #endif // DEBUG


        uint16_t frame_size = 6 + 4 * (DYNAMIC_NOSENSORS + 1);
        memoryData[0] = (uint8_t) frame_size >> 8;
        memoryData[1] = (uint8_t) frame_size & 0xFF;
        memoryData[2] = (uint8_t) ((enum algorithm) optimalset_en | (enum sampletype) measure_en << 2 | (enum dynamicSamplingType) threshold_en << 3);
        memoryData[3] = (uint8_t) DYNAMIC_NOSENSORS;
        memoryData[4] = (uint8_t) (SAMPLE_RATE_MS >> 8);
        memoryData[5] = (uint8_t) (SAMPLE_RATE_MS & 0xFF);
        memory_add(memoryData, 6);
        for (int i = 0; i < DYNAMIC_NOSENSORS; i++)
        {
            floatToByteArray(samples_for_measurement[i], memoryData);
            memory_add(memoryData, 4);
        }
        floatToByteArray(measurement, memoryData);
        memory_add(memoryData, 4);
        return;
    }
    else if ((sample < predicted - boundFactor) || (sample > predicted + boundFactor)){
        #ifdef DEBUG
            printk("Sampled outside boundary: Lower: %.3f, sample: %.3f, upper: %.3f\n\r", predicted - boundFactor, sample, predicted + boundFactor);
        #endif // DEBUG
        temp_Measure_Read(samples_for_measurement, AVG_1, DYNAMIC_NOSENSORS);

        measurement = optimalSet(samples_for_measurement, DYNAMIC_NOSENSORS);
        // measurement = average(samples_for_measurement, DYNAMIC_NOSENSORS);
        // measurement = simpleVoting(samples_for_measurement, DYNAMIC_NOSENSORS);
        #ifdef DEBUG
            printk("Measurement: %.3f\n\r", measurement);
        #endif // DEBUG

        thresh_index = 0;

        thresh_lastMeasure = measurement;

        uint16_t frame_size = 6 + 4 * (DYNAMIC_NOSENSORS + 1);
        memoryData[0] = (uint8_t) frame_size >> 8;
        memoryData[1] = (uint8_t) frame_size & 0xFF;
        memoryData[2] = (uint8_t) ((enum algorithm) optimalset_en | (enum sampletype) measure_en << 2 | (enum dynamicSamplingType) threshold_en << 3);
        memoryData[3] = (uint8_t) DYNAMIC_NOSENSORS;
        memoryData[4] = (uint8_t) (SAMPLE_RATE_MS >> 8);
        memoryData[5] = (uint8_t) (SAMPLE_RATE_MS & 0xFF);
        memory_add(memoryData, 6);
        for (int i = 0; i < DYNAMIC_NOSENSORS; i++)
        {
            floatToByteArray(samples_for_measurement[i], memoryData);
            memory_add(memoryData, 4);
        }
        floatToByteArray(measurement, memoryData);
        memory_add(memoryData, 4);
        return;
    }
    else{
        #ifdef DEBUG
            printk("Sampled within boundary: Lower: %.3f, sample: %.3f, upper: %.3f\n\r", predicted - boundFactor, sample, predicted + boundFactor);
            printk("Index = %d\n\r", thresh_index);
        #endif // DEBUG
        uint16_t frame_size = 9;
        memoryData[0] = (uint8_t) frame_size >> 8;
        memoryData[1] = (uint8_t) frame_size & 0xFF;
        memoryData[2] = (uint8_t) ((enum algorithm) optimalset_en | (enum sampletype) sample_en << 2 | (enum dynamicSamplingType) threshold_en << 3);
        memoryData[3] = (uint8_t) (SAMPLE_RATE_MS >> 8);
        memoryData[4] = (uint8_t) (SAMPLE_RATE_MS & 0xFF);
        memory_add(memoryData, 5);
        floatToByteArray(sample, memoryData);
        memory_add(memoryData, 4);
        return;
    }
    #ifdef DEBUG
    printk("\n\r");
    #endif // DEBUG

    k_timer_start(&adaptive_sampling_timer, K_MSEC(SAMPLE_RATE_MS), K_FOREVER);
}

extern void bollinger_work_handler(struct k_work *work){
    uint16_t twait_ms;
    float estimates[DYNAMIC_N];
    float tempSamples[DYNAMIC_NOSENSORS];
    float dyn;
    float standardDeviation = 0;


    uint8_t memoryData[6];
    uint16_t frame_size = 7 + 4 * (DYNAMIC_NOSENSORS + 1) * DYNAMIC_N;
    memoryData[0] = (uint8_t) frame_size >> 8;
    memoryData[1] = (uint8_t) frame_size & 0xFF;
    memoryData[2] = (enum algorithm) kalman_en | (enum sampletype) measure_en << 2 | (enum dynamicSamplingType) bollinger_en << 3;
    memoryData[3] = (uint8_t) DYNAMIC_NOSENSORS;
    memoryData[4] = (uint8_t) DYNAMIC_N;
    memory_add(memoryData, 5);

    bollinger_bbmid = 0;
    #ifdef DEBUG
        printk("Bollinger bands work handler\n\r");
        printk("Measurements: ");
    #endif // DEBUG

    for (int i = 0; i < DYNAMIC_N; i++)
    {
        temp_Measure_Read(tempSamples, AVG_1, DYNAMIC_NOSENSORS);

        // estimates[i] = optimalSet(tempSamples, DYNAMIC_NOSENSORS);
        // estimates[i] = average(tempSamples, DYNAMIC_NOSENSORS);
        // estimates[i] = simpleVoting(tempSamples, DYNAMIC_NOSENSORS);
        estimates[i] = kalman(tempSamples, DYNAMIC_NOSENSORS);

        #ifdef DEBUG
            printk("%.3f ", estimates[i]);
        #endif // DEBUG

        bollinger_bbmid += estimates[i]/DYNAMIC_N;

        for (int j = 0; j < DYNAMIC_NOSENSORS; j++)
        {
            floatToByteArray(tempSamples[j], memoryData);
            memory_add(memoryData, 4);
        }

        floatToByteArray(estimates[i], memoryData);
        memory_add(memoryData, 4);

    }
    #ifdef DEBUG
        printk("\n\rBollinger bbmid: %.3f\n\r", bollinger_bbmid);
    #endif // DEBUG

    for (int i = 0; i < DYNAMIC_N; i++){
        standardDeviation += pow(estimates[i] - bollinger_bbmid, 2);
    }
    standardDeviation = sqrt(standardDeviation)/DYNAMIC_N;
    #ifdef DEBUG
        printk("Bollinger Standard deviation: %.3f\n\r", standardDeviation);
    #endif // DEBUG
    dyn = DYNAMIC_B * standardDeviation;
    #ifdef DEBUG
        printk("Dynamicity: %.3f\n\r", dyn);
    #endif // DEBUG
    twait_ms = (uint16_t)(T_WAIT_MIN_MS + (T_WAIT_MAX_MS - T_WAIT_MIN_MS)/(1 + pow(dyn, DYNAMIC_PSI)));
    #ifdef DEBUG
        printk("Time until next [ms]: %d\n\n\r", twait_ms);
    #endif // DEBUG


    k_timer_start(&adaptive_sampling_timer, K_MSEC(twait_ms), K_FOREVER);

    memoryData[0] = twait_ms >> 8;
    memoryData[1] = twait_ms & 0xff;
    memory_add(memoryData, 2);
}

extern void linearDis_work_handler(struct k_work *work){
    uint16_t twait_ms;
    float estimates[DYNAMIC_N];
    float tempSamples[DYNAMIC_NOSENSORS];
    float dyn;
    float m, b;
    
    uint8_t memoryData[6];
    uint16_t frame_size = 7 + 4 * (DYNAMIC_NOSENSORS + 1) * DYNAMIC_N;
    memoryData[0] = (uint8_t) (frame_size >> 8);
    memoryData[1] = (uint8_t) frame_size & 0xFF;
    memoryData[2] = (enum algorithm) averaging_en | (enum sampletype) measure_en << 2 | (enum dynamicSamplingType) lineardist_en << 3;
    memoryData[3] = (uint8_t) DYNAMIC_NOSENSORS;
    memoryData[4] = (uint8_t) DYNAMIC_N;
    memory_add(memoryData, 5);

    #ifdef DEBUG
        printk("Linear Distance work handler\n\r");
        printk("Measurements: ");
    #endif // DEBUG

    for (int i = 0; i < DYNAMIC_N; i++)
    {
        temp_Measure_Read(tempSamples, AVG_1, DYNAMIC_NOSENSORS);
        // estimates[i] = optimalSet(tempSamples, DYNAMIC_NOSENSORS);
        estimates[i] = average(tempSamples, DYNAMIC_NOSENSORS);
        // estimates[i] = simpleVoting(tempSamples, DYNAMIC_NOSENSORS);
        // estimates[i] = kalman(tempSamples, DYNAMIC_NOSENSORS);

        #ifdef DEBUG
            printk("%.3f ", estimates[i]);
        #endif // DEBUG

        for (int j = 0; j < DYNAMIC_NOSENSORS; j++)
        {
            floatToByteArray(tempSamples[j], memoryData);
            memory_add(memoryData, 4);
        }

        floatToByteArray(estimates[i], memoryData);
        memory_add(memoryData, 4);

    }

    m = (estimates[DYNAMIC_N-1] - estimates[0])/DYNAMIC_N;
    b = estimates[0];

    #ifdef DEBUG
        printk("\n\rm: %.3f, b: %.3f\n\r", m, b);
    #endif // DEBUG
    
    float lindist = 0;
    for (int i = 0; i < DYNAMIC_N; i++)
    {
        lindist += fabs(estimates[i] - ((m * i) + b))/DYNAMIC_N;
    }

    #ifdef DEBUG
        printk("Mean linear distances: %.3f\n\r", lindist);
    #endif // DEBUG

    dyn = lindist * DYNAMIC_B/2;

    #ifdef DEBUG
        printk("Dynamicity: %.3f\n\r", dyn);
    #endif // DEBUG

    twait_ms = (uint16_t)(T_WAIT_MIN_MS + (T_WAIT_MAX_MS - T_WAIT_MIN_MS)/(1 + pow(dyn, DYNAMIC_PSI)));

    #ifdef DEBUG
        printk("Time until next[ms]: %d\n\n\r", twait_ms);
    #endif // DEBUG
    
    // TODO Store measurement frame

    k_timer_start(&adaptive_sampling_timer, K_MSEC(twait_ms), K_FOREVER);
    
    memoryData[0] = twait_ms >> 8;
    memoryData[1] = twait_ms & 0xff;
    memory_add(memoryData, 2);  
}

extern void onlyAccAlg_work_handler(struct k_work *work){
    uint16_t twait_ms;
    float estimate = 0.0;
    float tempSamples[DYNAMIC_NOSENSORS];

    uint8_t memoryData[6];
    uint16_t frame_size = 4 + 4 * (DYNAMIC_NOSENSORS + 1);
    memoryData[0] = (uint8_t) (frame_size >> 8);
    memoryData[1] = (uint8_t) frame_size & 0xFF;
    memoryData[2] = (enum algorithm) kalman_en | (enum sampletype) sample_en << 2 | (enum dynamicSamplingType) onlyAccAlg_en << 3;
    memoryData[3] = (uint8_t) DYNAMIC_NOSENSORS;
    memory_add(memoryData, 4);

    #ifdef DEBUG
        printk("Only accuracy algorithm work handler: \n\r");
        printk("Measurements: ");
    #endif // DEBUG


    temp_Measure_Read(tempSamples, AVG_1, DYNAMIC_NOSENSORS);
    
    // estimate = optimalSet(tempSamples, DYNAMIC_NOSENSORS);
    // estimate = average(tempSamples, DYNAMIC_NOSENSORS);
    // estimate = simpleVoting(tempSamples, DYNAMIC_NOSENSORS);
    estimate = kalman(tempSamples, DYNAMIC_NOSENSORS);

    #ifdef DEBUG
        printk("%.3f ", estimate);
    #endif // DEBUG

    for (int j = 0; j < DYNAMIC_NOSENSORS; j++)
    {
        floatToByteArray(tempSamples[j], memoryData);
        memory_add(memoryData, 4);
    }

    floatToByteArray(estimate, memoryData);
    memory_add(memoryData, 4);
}