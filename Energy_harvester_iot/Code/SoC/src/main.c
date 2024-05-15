#include <zephyr/kernel.h>
#include "inc.h"
#include <nrfx_config.h>
#include "sdk_config.h"


// I2C variables
extern nrfx_twim_xfer_desc_t twim_xfer_desc;


uint8_t gpio_portMask = 0x00;
uint8_t switch_portMask = 0x00;


int main(void)
{
        i2c_init();
        GPIO_setDir(GPIO_BASE_PORTDIR);
        return 0;
}
