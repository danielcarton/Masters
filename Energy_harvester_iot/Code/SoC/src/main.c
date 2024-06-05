#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>

#include "inc.h"

#define USERNODE DT_PATH(zephyr_user)

const struct gpio_dt_spec raspiCom_in = GPIO_DT_SPEC_GET(USERNODE, raspi_comm_in_gpios);
const struct gpio_dt_spec raspiCom_out = GPIO_DT_SPEC_GET(USERNODE, raspi_comm_out_gpios);

extern uint32_t write_address;

uint8_t gpio_portMask = 0x00;
uint8_t switch_portMask = 0x00;

static struct gpio_callback raspi_comm_cb;

static struct k_work test_start_work;
static struct k_work test_over_work;


void raspi_comm_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
        printk("test handler called\n\r");

        printk("Input pin level: %d\n\r", gpio_pin_get_dt(&raspiCom_in));

        if (gpio_pin_get_dt(&raspiCom_in)){
                // Test started
                k_work_submit(&test_start_work);
        }


        if (!gpio_pin_get_dt(&raspiCom_in)){
                // Test ended
                // stop timers
                timer_stop_ad();
                k_work_submit(&test_over_work);
        }
        printk("\n\r");
}

static void test_start_work_handler(struct k_work *work){
        printk("Test is starting\n\r");
        // take memory back
        gpio_portMask |= 0x01;
        GPIO_setOutput(gpio_portMask);

        // clear memory pin
        gpio_pin_set_dt(&raspiCom_out, 0);

        // reset write address
        write_address = 0;

        // start timer
        // timer_init_threshold_ad();
        // timer_init_bollinger_ad();
        // timer_init_linearDis_ad();
        timer_init_onlyAccAlg();
}

static void test_over_work_handler(struct k_work *work){
        printk("Test is ending\n\r");

        // activate memory bus
        switch_portMask = 0x01;
        I2C_switch_setChannel(switch_portMask);

        // make sure you have memory
        gpio_portMask |= 0x01;
        GPIO_setOutput(gpio_portMask);

        // add last to memory
        uint8_t data[MEMORY_BLOCK_SIZE] = {0};
        memory_add(&data, MEMORY_BLOCK_SIZE);

        // disable all buses
        switch_portMask = 0x00;
        I2C_switch_setChannel(switch_portMask);

        // disable all load switches & give memory
        gpio_portMask = 0x00;
        GPIO_setOutput(gpio_portMask);

        // set memory pin
        gpio_pin_set_dt(&raspiCom_out, 1);
}

int main(void)
{

        // printk("Starting real test\n\r");

        // init devices
        GPIO_init();
        I2C_switch_init();

        // set GPIO port direction
        GPIO_setDir(GPIO_BASE_PORTDIR);

        I2C_switch_setChannel(0x00);
        GPIO_setOutput(0x00);

        // Configure GPIO pins
        gpio_pin_configure_dt(&raspiCom_in, GPIO_INPUT);
        gpio_pin_configure_dt(&raspiCom_out, GPIO_OUTPUT);

        // Initialize work with their handlers
        k_work_init(&test_start_work, test_start_work_handler);
        k_work_init(&test_over_work, test_over_work_handler);

        // attach interrupts to input pin
        gpio_pin_interrupt_configure_dt(&raspiCom_in, GPIO_INT_EDGE_BOTH);
        gpio_init_callback(&raspi_comm_cb, raspi_comm_interrupt_handler, BIT(raspiCom_in.pin)); 	
        gpio_add_callback(raspiCom_in.port, &raspi_comm_cb);

        while (1)
        {
                k_msleep(1000*10*60);
        }
        

        return 0;
}














// printk("Giving access to memory awway\n\r");
// GPIO_init();
// I2C_switch_init();
// GPIO_setDir(GPIO_BASE_PORTDIR);
// GPIO_setOutput(0x00);
// while (1)
// {
//         gpio_pin_toggle_dt(&raspiCom_out);
//         k_msleep(1000);
// }

// printk("Starting memory test\n\r");
// uint8_t uart_tx_buf[3];
// GPIO_init();
// I2C_switch_init();
// GPIO_setDir(GPIO_BASE_PORTDIR);
// GPIO_setOutput(1);
// I2C_switch_setChannel(0x01);
// uart_configure(uart0_dev, &uart_cfg);

// while (1)
// {
//         for (int j = 0; j < 100; j++)
//         {
//                 for (int i = 0; i < 256; i++)
//                 {
//                         uint8_t temp = i;
//                         uart_tx_buf[1] = temp;
//                         uart_tx_buf[0] = (uint8_t) j;
//                         uart_tx(uart0_dev, &uart_tx_buf, 2, SYS_FOREVER_US);
//                         memory_add(&temp, 1);
//                         k_msleep(1);
//                 }
//         }
// }



// uint8_t uart_tx_buf[]= "Testing UART!";
// uart_configure(uart0_dev, &uart_cfg);
// printk("Starting UART test\n\r");
// while (1)
// {
//         uart_tx(uart0_dev, uart_tx_buf, 14, SYS_FOREVER_US);
//         k_msleep(2000);
// }


// printk("Starting adaptive sampling test\n\r");
// GPIO_init();
// I2C_switch_init();
// GPIO_setDir(GPIO_BASE_PORTDIR);
// gpio_portMask = 1;
// GPIO_setOutput(gpio_portMask);
// switch_portMask = 1;
// I2C_switch_setChannel(switch_portMask);
// // timer_init_threshold_ad();
// // timer_init_bollinger_ad();
// timer_init_linearDis_ad();
// while (1)
// {
//         k_msleep(1000*60);
// }


// printk("Starting test\n\r");
// GPIO_init();
// I2C_switch_init();
// GPIO_setDir(GPIO_BASE_PORTDIR);
// I2C_switch_setChannel(0x00);

// float output[6];
// while(1){
//         temp_Measure_Read(output, AVG_1, 6);
//         printk("Data output from sensors is: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f. Optimal Set accuracy improvement: %.4f\n\r", output[0], output[1], output[2], output[3], output[4], output[5], optimalSet(output, 6));
//         optimalSet(output, 4);
//         k_msleep(100);
// }
