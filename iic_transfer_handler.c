#include "iic_transfer_handler.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include <string.h>


/* TWI instance ID. */
#define TWI_INSTANCE_ID 0

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

static uint8_t iic_sendbuf[20];
static uint8_t iic_recvbuf[20];

ret_code_t err_code;

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context)
{
	switch (p_event->type)
	{
	case NRF_DRV_TWI_EVT_DONE:
		m_xfer_done = true;
		break;
	default:
		break;
	}
}

void twi_init(void)
{
	
	const nrf_drv_twi_config_t twi_afe_config = {
		.scl = BOARD_SCL_PIN,
		.sda = BOARD_SDA_PIN,
		.frequency = NRF_DRV_TWI_FREQ_100K,
		.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
		.clear_bus_init = false
	};

	err_code = nrf_drv_twi_init(&m_twi, &twi_afe_config, twi_handler, NULL);
	APP_ERROR_CHECK(err_code);

	nrf_drv_twi_enable(&m_twi);
}

void twi_scanner(void)
{
		uint8_t address;
    uint8_t sample_data = 0xff;
    bool detected_device = false;
	
		for (address = 1; address <= 127; address++)
    {
        err_code = nrf_drv_twi_tx(&m_twi, address, &sample_data, 1, false);
        if (err_code == NRF_SUCCESS)
        {
            detected_device = true;
            NRF_LOG_INFO("TWI device detected at address 0x%x.", address);
        }
        NRF_LOG_FLUSH();
    }

    if (!detected_device)
    {
        NRF_LOG_INFO("No device was found.");
        NRF_LOG_FLUSH();
    }

}

void twi_readRegisters(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len){
	
	iic_sendbuf[0] = reg;
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, addr, iic_sendbuf, 1, true);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false) __WFE();
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_rx(&m_twi, addr, iic_recvbuf, len);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false) __WFE();
	
	memcpy(buffer, iic_recvbuf, len);

}

void twi_readRegisters_delay(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len, int delay){
	
	iic_sendbuf[0] = reg;
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, addr, iic_sendbuf, 1, true);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false) __WFE();
	
	nrf_delay_ms(delay);
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_rx(&m_twi, addr, iic_recvbuf, len);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false) __WFE();
	
	memcpy(buffer, iic_recvbuf, len);

}

void twi_readRegisters_stop(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len){
	
	iic_sendbuf[0] = reg;
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, addr, iic_sendbuf, 1, false);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false) __WFE();
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_rx(&m_twi, addr, iic_recvbuf, len);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false) __WFE();
	
	memcpy(buffer, iic_recvbuf, len);

}

void twi_writeRegisters(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len){
	
	iic_sendbuf[0] = reg;
	memcpy(iic_sendbuf + 1, buffer, len);
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, addr, iic_sendbuf, len + 1, false);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false) __WFE();

}

///////////////////VL53L1X

void twi_vl53l1x_write(uint8_t addr, uint16_t index, uint8_t data){
	
	iic_sendbuf[0] = index >> 8;
	iic_sendbuf[1] = index;
	iic_sendbuf[2] = data;
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, addr, iic_sendbuf, 3, false);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false) __WFE();

}

void twi_vl53l1x_read(uint8_t addr, uint16_t index, uint8_t *data){
	
	iic_sendbuf[0] = index >> 8;
	iic_sendbuf[1] = index;
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, addr, iic_sendbuf, 2, false);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false) __WFE();
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_rx(&m_twi, addr, iic_recvbuf, 1);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false) __WFE();
	
	*data = iic_recvbuf[0];

}

void twi_vl53l1x_seq_write(uint8_t addr, uint16_t index, uint8_t *data, uint16_t length){
	
	iic_sendbuf[0] = index >> 8;
	iic_sendbuf[1] = index;
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, addr, iic_sendbuf, 2, false);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false) __WFE();
	
	memcpy(iic_sendbuf, data, length);
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, addr, iic_sendbuf, length, false);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false) __WFE();
	
}

void twi_vl53l1x_seq_read(uint8_t addr, uint16_t index, uint8_t *data, uint16_t length){
	
	iic_sendbuf[0] = index >> 8;
	iic_sendbuf[1] = index;
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, addr, iic_sendbuf, 2, false);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false) __WFE();
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_rx(&m_twi, addr, iic_recvbuf, length);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false) __WFE();
	
	memcpy(data, iic_recvbuf, length);

}

