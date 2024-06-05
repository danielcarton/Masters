#ifndef _TIMERS_H_
#define _TIMERS_H_

// shortest period of time between adaptive sampling phases
#define T_WAIT_MIN_MS 1000

// longest period of time between adaptive sampling phases
#define T_WAIT_MAX_MS 10000

// Milliseconds between samples for regular sampling and threshold-based sampling
#define SAMPLE_RATE_MS 1000

// boundary of upper and lower threshold
#define THRESHOLD_SIZE 1

#define DYNAMIC_N 16
#define DYNAMIC_NOSENSORS 1
#define DYNAMIC_PSI 2
#define DYNAMIC_B 50


// threshold
void timer_init_threshold_ad(void);
extern void threshold_ad(struct k_timer *timer);
extern void threshold_work_handler(struct k_work *work);

// bollinger bands
void timer_init_bollinger_ad(void);
extern void bollinger_ad(struct k_timer *timer);
extern void bollinger_work_handler(struct k_work *work);

// linear distance
void timer_init_linearDis_ad(void);
extern void linearDis_ad(struct k_timer *timer);
extern void linearDis_work_handler(struct k_work *work);

// only accuracy algorithms
void timer_init_onlyAccAlg(void);
extern void onlyAccAlg(struct k_timer *timer);
extern void onlyAccAlg_work_handler(struct k_work *work);


extern void my_work_handler(struct k_work *work);



void timer_stop_ad(void);

typedef enum algorithm{
    optimalset_en = 0b00,
    simplevoting_en = 0b01,
    averaging_en = 0b10,
    kalman_en = 0b11
};

typedef enum sampletype{
    sample_en = 0b0,
    measure_en = 0b1
};

typedef enum dynamicSamplingType{
    threshold_en = 0b00,
    bollinger_en = 0b01,
    lineardist_en = 0b10,
    onlyAccAlg_en = 0b11
};

#endif // !_TIMERS_H_