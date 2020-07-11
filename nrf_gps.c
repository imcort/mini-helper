#include "nrf_gps.h"

#define UART_TX_BUF_SIZE 10  /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 255 /**< UART RX buffer size. */

#define GPS_RX_PIN 29
#define GPS_TX_PIN 31
#define GPS_EN_PIN 3

static uint8_t     rx_buf[UART_RX_BUF_SIZE];                                                  
static uint8_t     tx_buf[UART_TX_BUF_SIZE];  

static int32_t gps_time = 0;      //UNIX timestamp
static bool gps_valid = false;
static int32_t gps_latitude = 0;  //+-2147483648 --> 180.0000000e7 percision 1cm
static int32_t gps_longitude = 0; //+-2147483648 --> 180.0000000e7 percision 1cm
static uint16_t gps_speed = 0;    //65535 --> 500.00 m/s
static int16_t gps_course = 0;    //32768 --> 360.0 deg
static int16_t gps_variation = 0;    //32768 --> 360.0 deg

//struct minmea_time time;
//    bool valid;
//    struct minmea_float latitude;
//    struct minmea_float longitude;
//    struct minmea_float speed;
//    struct minmea_float course;
//    struct minmea_date date;
//    struct minmea_float variation;


//static int gps_fix_quality = 0;
//static int gps_total_sats = 0;

/* GPS module don't need to use TX if only receive data*/

void uart_event_handle(app_uart_evt_t *p_event)
{
  static char data_array[UART_RX_BUF_SIZE];
  static uint8_t index = 0;
  uint32_t err_code;

  switch (p_event->evt_type)
  {
  case APP_UART_DATA_READY:
    UNUSED_VARIABLE(app_uart_get((uint8_t *)&data_array[index]));
    index++;

    if ((data_array[index - 1] == '\n') || (index == UART_RX_BUF_SIZE))
    {
      data_array[index] = '\0';

      if (index > 1)
      {
        //NRF_LOG_INFO("%s", data_array);
        switch (minmea_sentence_id(data_array, false))
        {
        case MINMEA_SENTENCE_RMC:
        {
          struct minmea_sentence_rmc frame;
          if (minmea_parse_rmc(&frame, data_array))
          {
						if(frame.valid){
							gps_valid = true;
							struct timespec ts;
							if(minmea_gettime(&ts, &frame.date, &frame.time))
								gps_time = ts.tv_sec;
							gps_latitude = minmea_rescale(&frame.latitude, 10000000);
							gps_longitude = minmea_rescale(&frame.longitude, 10000000);
							gps_speed = minmea_rescale(&frame.speed, 100);
							gps_course = minmea_rescale(&frame.course, 10);
							gps_variation = minmea_rescale(&frame.variation, 10);
						} else {
							
							gps_valid = false;
						
						}
						
            
          }
        }
        break;
        default:
          break;
        }
      }

      index = 0;
    }
    break;

  default:
    break;
  }
}

void gps_init(void)
{
	nrf_gpio_cfg_output(GPS_EN_PIN);
	nrf_gpio_pin_clear(GPS_EN_PIN);
}

void gps_on(void)
{
	ret_code_t err_code;
	
	app_uart_comm_params_t const comm_115200_params =
	{
			.rx_pin_no    = GPS_RX_PIN,
			.tx_pin_no    = GPS_TX_PIN,
			.rts_pin_no   = UART_PIN_DISCONNECTED,
			.cts_pin_no   = UART_PIN_DISCONNECTED,
			.flow_control = APP_UART_FLOW_CONTROL_DISABLED,
			.use_parity   = false,
			.baud_rate    = NRF_UART_BAUDRATE_115200
	};
	
	app_uart_buffers_t buffers = {
	
			.rx_buf      = rx_buf,                                                            
			.rx_buf_size = sizeof (rx_buf),                                                
			.tx_buf      = tx_buf,                 
			.tx_buf_size = sizeof (tx_buf)
	
	};
	
	err_code = app_uart_init(&comm_115200_params, &buffers, uart_event_handle, APP_IRQ_PRIORITY_LOWEST);
	
	APP_ERROR_CHECK(err_code);
	
	nrf_gpio_pin_set(GPS_EN_PIN);
		
}

void gps_off(void)
{
	
	nrf_gpio_pin_clear(GPS_EN_PIN);
	app_uart_close();
	
}

int32_t gps_gettime(){
	
	return gps_time;

}

bool gps_is_valid(){

	return gps_valid;

}
