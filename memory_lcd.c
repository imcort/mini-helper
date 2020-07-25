#include "memory_lcd.h"
#include <stdlib.h>
#include "string.h"

#include "lvgl.h"
#include "app_timer.h"
#include "nrf_log.h"

#define SPI_INSTANCE  1
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
static volatile bool spi_xfer_done = false;

#define SHARPMEM_BIT_WRITECMD (0x01)
#define SHARPMEM_BIT_VCOM (0x02)
#define SHARPMEM_BIT_CLEAR (0x04)
#define TOGGLE_VCOM                                                            \
  do {                                                                         \
    _sharpmem_vcom = _sharpmem_vcom ? 0x00 : SHARPMEM_BIT_VCOM;                \
  } while (0);
	
#define DISP_SS_LOW     nrf_gpio_pin_clear(28)
#define DISP_SS_HIGH    nrf_gpio_pin_set(28)

static uint8_t sharpmem_buffer[22];	
uint8_t _sharpmem_vcom;
	
static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];
	
APP_TIMER_DEF(lvgl_timer);
#define LVGL_TICK_PERIOD 1
	
void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
	
	spi_xfer_done = true;
	
}

static void lvgl_timer_handler(void *p_context)
{
	
  lv_tick_inc(LVGL_TICK_PERIOD);
}

void disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
	uint8_t* pointer = &(color_p->full);
  for (int y = area->y1; y <= area->y2; y++)
  {
		disp_refresh(y, pointer);
		pointer += 144;
  }
  lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
}

void my_rounder_cb(lv_disp_drv_t * disp_drv, lv_area_t * area)
{
  /* Update the areas as needed. Can be only larger.
   * For example to always have lines 8 px height:*/
   area->x1 = 0;
   area->x2 = 143;
}

void my_printf(lv_log_level_t level, const char * file, int line, const char * fn_name, const char * dsc)
{
	NRF_LOG_INFO("level:%d,file:%s,line:%d,dsc:%s",level,file,line,dsc);
}

void disp_spi_init(){
	
	nrf_gpio_cfg_output(28);
	DISP_SS_LOW;
	
	  nrf_drv_spi_config_t spi_config = {                        \
			.sck_pin      = 30,                						 \
			.mosi_pin     = 29,               						 \
			.miso_pin     = NRF_DRV_SPI_PIN_NOT_USED,               						 \
			.ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,                \
			.irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY,         \
			.orc          = 0x00,                                    \
			.frequency    = NRF_DRV_SPI_FREQ_8M,                     \
			.mode         = NRF_DRV_SPI_MODE_0,                      \
			.bit_order    = NRF_DRV_SPI_BIT_ORDER_LSB_FIRST,         \
		};
	  
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

}

void disp_clear(void){
	
	memset(sharpmem_buffer, 0xff, 22);
	sharpmem_buffer[0] = (_sharpmem_vcom | SHARPMEM_BIT_CLEAR);
	TOGGLE_VCOM;
	
	DISP_SS_HIGH;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, sharpmem_buffer, 2, NULL, 0));
	while(!spi_xfer_done) __WFE();
	DISP_SS_LOW;

}

bool disp_init(void){
	
	ret_code_t err_code;
	
	disp_spi_init();
	_sharpmem_vcom = SHARPMEM_BIT_VCOM;
	
	err_code = app_timer_create(&lvgl_timer, APP_TIMER_MODE_REPEATED, lvgl_timer_handler);
  APP_ERROR_CHECK(err_code);
	
	disp_clear();
	
	lv_init();
	
  lv_log_register_print_cb(my_printf); /* register print function for debugging */

  lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);

  /*Initialize the display*/
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 144;
  disp_drv.ver_res = 168;
  disp_drv.flush_cb = disp_flush;
	disp_drv.rounder_cb = my_rounder_cb;
  disp_drv.buffer = &disp_buf;
  disp_drv.rotated = 0;
  lv_disp_drv_register(&disp_drv);
	
  err_code = app_timer_start(lvgl_timer, APP_TIMER_TICKS(LVGL_TICK_PERIOD), NULL);
  APP_ERROR_CHECK(err_code);
	
	return true;
	
}



static const uint8_t set[] = {1, 2, 4, 8, 16, 32, 64, 128},
                             clr[] = {(uint8_t)~1,  (uint8_t)~2,  (uint8_t)~4,
                                      (uint8_t)~8,  (uint8_t)~16, (uint8_t)~32,
                                      (uint8_t)~64, (uint8_t)~128};

void disp_refresh(uint8_t startLine, uint8_t* buffer){
	
	sharpmem_buffer[0] = (_sharpmem_vcom | SHARPMEM_BIT_WRITECMD);
	TOGGLE_VCOM;
	sharpmem_buffer[1] = startLine + 1;
	
	for(int i=0;i<144;i++){
		
		if(buffer[i]){
			
			sharpmem_buffer[(0 * 160 + i + 16) / 8] |= set[i & 7];
		
		} else {
		
			sharpmem_buffer[(0 * 160 + i + 16) / 8] &= clr[i & 7];
		
		}
	
	}
	
	DISP_SS_HIGH;
	
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, sharpmem_buffer, 22, NULL, 0));
	while(!spi_xfer_done) __WFE();
	
	
	DISP_SS_LOW;

}

void disp_show(){
	
	sharpmem_buffer[0] = (_sharpmem_vcom);
	TOGGLE_VCOM;
	
	DISP_SS_HIGH;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, sharpmem_buffer, 2, NULL, 0));
	while(!spi_xfer_done) __WFE();
	DISP_SS_LOW;

}
