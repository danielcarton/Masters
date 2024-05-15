#include "I2C.h"
#include <nrfx_twim.h>


nrfx_twim_xfer_desc_t twim_xfer_desc = NRFX_TWIM_XFER_DESC_TX(NULL,
                                                                NULL,
                                                                NULL);

nrfx_twim_t twim_inst = NRFX_TWIM_INSTANCE(TWIM_INST_IDX);
nrfx_twim_config_t twim_config = {
    .scl_pin = TWIM_SCL_PIN,
    .sda_pin = TWIM_SDA_PIN,
    .frequency = NRF_TWIM_FREQ_100K,    //  TODO test with other frequencies
    .interrupt_priority = NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY,
    .hold_bus_uninit    = false
};

// Initialize I2C
void i2c_init(void){
    nrfx_err_t status;
    (void)status;

    status = nrfx_twim_init(&twim_inst, &twim_config, NULL, &twim_xfer_desc);
    nrfx_twim_enable(&twim_inst);

}

// Write data to I2C
void i2c_write(uint8_t addr, uint8_t *data, uint8_t len){
    nrfx_err_t status;
    (void)status;    
    twim_xfer_desc.address = addr;
    twim_xfer_desc.p_primary_buf = data;
    twim_xfer_desc.primary_length = len;
    twim_xfer_desc.type = NRFX_TWIM_XFER_RX;
    status = nrfx_twim_xfer(&twim_inst, &twim_xfer_desc, 0);
}

// Read data from I2C
void i2c_read(uint8_t addr, uint8_t *data, uint8_t len){
    nrfx_err_t status;
    (void)status;
    twim_xfer_desc.address = addr;
    twim_xfer_desc.p_primary_buf = data;
    twim_xfer_desc.primary_length = len;
    twim_xfer_desc.type = NRFX_TWIM_XFER_TX;
    status = nrfx_twim_xfer(&twim_inst, &twim_xfer_desc, 0);
}
