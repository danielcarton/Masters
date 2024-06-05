#include <zephyr/kernel.h>
#include "inc.h"
#include <nrfx_config.h>


// I2C variables
extern nrfx_twim_xfer_desc_t twim_xfer_desc;


u8 gpio_portMask = 0x00;
u8 switch_portMask = 0x00;


int main(void)
{
        // i2c_init();
        // GPIO_setDir(GPIO_BASE_PORTDIR);
        return 0;
}
