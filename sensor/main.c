#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <stdio.h>
#include <zephyr/sys_clock.h>
#include "inc/bme680_reg.h"

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

#define MSG_SIZE 32

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

//below code is about sensors

#define BME680_ADDR     0x77
#define veml7700        0x10

#define VEML7700_High_Resolution_Output_Data 0x04

#define COMMAND_CODE    0x00
#define MY_STACK_SIZE   5000

#define SLEEP_TIME_MS  3000

int evt=1;// evt is used if temp is over 30
char temp[20];
char hum[20];
char lightresult[20];
char soil[20];

// Temperature threshold
#define TEMPERATURE_THRESHOLD 30

struct k_queue queue; 

// typedef enum _event_t 
// {
//     b1_evt = 0,
//     b2_evt = 1,
//     b3_evt = 2,
//     no_evt = 3
// } event_t;

//
/*
	# The light sensor 
Natural Light Condition	    Typical Lux
Direct Sunlight	            32,000 to 100,000
Ambient Daylight	        10,000 to 25,000
Overcast Daylight	        1000
Sunset & Sunrise	        400
Moonlight (Full moon)	    1
Night (No moon)	            < 0.01

	# the moisture sensor value description
	# 0  ~300     dry soil
	# 300~700     humid soil
	# 700~950     in water
*/

/* For i2c variable */
const struct device *i2c_dev;

uint8_t  data[3];
uint8_t h_data[2]; //humidity register
uint32_t temp_adc, humidity_adc;
uint8_t  par_t[5];
uint8_t par_h[9];//humidity register
int32_t p1,p2,p3;
int32_t h1,h2,h3,h4,h5,h6,h7;
int32_t temp_comp;
uint8_t light[2];


#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

/* For ADC variable */
int err;
uint16_t buf;

struct adc_sequence sequence = {

	.buffer = &buf,
	/* buffer size in bytes, not number of samples */
	.buffer_size = sizeof(buf),

};
/* End of ADC variable */

int32_t temp_convert(uint32_t temp_adc, int32_t p1, int32_t p2, int32_t p3){
    
	int32_t var1 = ((int32_t)temp_adc >> 3) - ((int32_t)p1<< 1);
    int32_t var2 = (var1* (int32_t)p2) >> 11;
    int32_t var3 = ((((var1 >> 1) * (var1 >> 1)) >> 12) * ((int32_t)p3 << 4)) >> 14;
    int32_t t_fine = var2 + var3;
    int32_t temp_comp = (int32_t)((t_fine * 5) + 128) >> 8;
    return temp_comp;

}

int32_t humidity_convert(int32_t temp_comp, uint32_t humidity_adc, int32_t h1, int32_t h2, int32_t h3, int32_t h4, int32_t h5, int32_t h6, int32_t h7){
    
	int32_t temp_scaled=temp_comp;
    int32_t var1 = (int32_t)humidity_adc - (int32_t)(h1<<4) - (((temp_scaled*h3)/((int32_t)100))>>1);
    int32_t var2 = (h2*(((temp_scaled*h4)/((int32_t)100))+(((temp_scaled*((temp_scaled*h5)/((int32_t)100)))>>6)/((int32_t)100))+((int32_t)(1<<14))))>>10;
    int32_t var3 = var1*var2;
    int32_t var4 = ((h6<<7)+((temp_scaled*h7)/((int32_t)100)))>>4;
    int32_t var5 = ((var3 >> 14)* (var3 >>14))>>10;
    int32_t var6 =(var4 *var5)>>1;
    int32_t hum_comp = (((var3 +var6)>>10)*((int32_t)1000))>>12;
    return hum_comp;

}

void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	while (uart_irq_rx_ready(uart_dev)) {

		uart_fifo_read(uart_dev, &c, 1);

		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
}


void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++)
	{
		uart_poll_out(uart_dev, buf[i]);
	}
}

void svf_i2c_init(void){

	if (i2c_dev == NULL || !device_is_ready(i2c_dev)) {
		printk("I2C: Device driver not found.\n");
		return;
	}
	else{
		printk("I2C: BME 680, VEML 7700\n");
	}

    //tempurature register
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,0xE9,&par_t[0]);
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,0XEA,&par_t[1]);
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,0x8A,&par_t[2]);
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,0x8B,&par_t[3]);
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,0x8c,&par_t[4]);

    p1=(int32_t) ((uint16_t)par_t[0] | ((uint16_t)par_t[1])<<8);
    p2=(int32_t)((uint16_t)par_t[2] | ((uint16_t)par_t[3])<<8);
    p3=(int32_t)par_t[4];

    //humidity register
    //i2c_reg_read_byte(i2c_dev, BME680_ADDR,0xE2,&par_h[0]);//vary
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,0xE3,&par_h[1]);
    //i2c_reg_read_byte(i2c_dev, BME680_ADDR,0xE2,&par_h[2]);//vary
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,0xE1,&par_h[3]);
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,0xE4,&par_h[4]);
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,0xE5,&par_h[5]);
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,0xE6,&par_h[6]);
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,0xE7,&par_h[7]);
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,0xE8,&par_h[8]);

    // h1=(int32_t)((uint16_t)par_h[0] | (uint16_t)par_h[1]<<8);
    // h2=(int32_t)((uint16_t)par_h[2] | (uint16_t)par_h[3]<<8);
    h3=(int32_t)par_h[4];
    h4=(int32_t)par_h[5];
    h5=(int32_t)par_h[6];
    h6=(int32_t)par_h[7];
    h7=(int32_t)par_h[8];

}

void svf_adc_init(void){

	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {

		if (!device_is_ready(adc_channels[i].dev)) {

			printk("ADC controller device not ready\n");
			return;

		}

		err = adc_channel_setup_dt(&adc_channels[i]);

		if (err < 0) {

			printk("Could not setup channel #%d (%d)\n", i, err);
			return;

		}
	}

}

void svf_i2c_loop(void){

	int ret1=i2c_reg_write_byte(i2c_dev, BME680_ADDR,BME680_CTRL_HUM,0b001);//force mode: 01	
    i2c_reg_write_byte(i2c_dev, BME680_ADDR,BME680_CTRL_MEAS,0b010 << 5 | 0b01);//force mode: 01
	if (ret1 != 0) {
        printk("Failedbme\n");
        //continue;
    }
    //tempuratue below
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,BME680_TEMP_MSB,&data[0]);
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,BME680_TEMP_LSB,&data[1]);
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,BME680_TEMP_XLSB,&data[2]); 

    temp_adc=((uint32_t)data[0])<<12 | ((uint32_t)data[1])<<4 | ((uint32_t)data[2])>>4;
    temp_comp=temp_convert(temp_adc,p1,p2,p3);
    //printk("T: %d (degrees Celsius)\n", temp_comp/100);
	sprintf(temp, "%d", temp_comp/100);//convert temp value into string type

	//generate an interrupt
	if((temp_comp/100) > TEMPERATURE_THRESHOLD){
		
		
		int err = k_queue_alloc_append(&queue, &evt);
			if (err == 0) {
				//printk("put event in the queue");
			} else if (err == -ENOMEM) {
				printk("Error: insufficient memory in the caller's resource pool");
			} else {
				printk("Error: unable to append event to the queue");
			}
	}



    //humidity below
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,0xE2,&par_h[0]);//vary
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,0xE2,&par_h[2]);//vary
    h1=(int32_t)(((uint16_t)par_h[0]&0b1111) | ((uint16_t)par_h[1]<<4));
    h2=(int32_t)(((uint16_t)par_h[2]>>4) | ((uint16_t)par_h[3]<<4));
        
    // BME680_HUM_LSB      0x26
    // BME680_HUM_MSB      0x25 
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,BME680_HUM_LSB,&h_data[0]);
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,BME680_HUM_MSB,&h_data[1]);

    humidity_adc = ((uint32_t)h_data[0]) | ((uint32_t)h_data[1])<<8 ;

    //printk("H: %d (\%)\n", humidity_convert(temp_comp,humidity_adc,h1,h2,h3,h4,h5,h6,h7)/1000);
	int i=humidity_convert(temp_comp,humidity_adc,h1,h2,h3,h4,h5,h6,h7)/1000;
	sprintf(hum, "%d", i);//convert hum value into string type

    //read light value
    int ret=i2c_reg_write_byte(i2c_dev, veml7700, COMMAND_CODE, 0x00);//mode
        
    if (ret != 0) {
        printk("Failed to write command to VEML\n");
        //continue;
    }
	
    i2c_burst_read(i2c_dev,veml7700,VEML7700_High_Resolution_Output_Data,(uint8_t*)&light,2);
    int16_t lightvalue=((int16_t)light[1]<<8) | (int16_t)light[0];//1 0
    //printk("L: %d\n",lightvalue);
	sprintf(lightresult, "%d", lightvalue);//convert light value into string type

}

void svf_adc_loop(void){

	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
			
		int32_t val_mv;
		int32_t sen_value;

		/* For Debugger */
		/*
		printk("INFO: - %s, channel %d\n",
			    adc_channels[i].dev->name,
			    adc_channels[i].channel_id);
		*/

		(void)adc_sequence_init_dt(&adc_channels[i], &sequence);

		err = adc_read(adc_channels[i].dev, &sequence);

		if (err < 0) {

			printk("Could not read (%d)\n", err);
			continue;

		}

			/*
			 * If using differential mode, the 16 bit value
			 * in the ADC sample buffer should be a signed 2's
			 * complement value.
			 */

		if (adc_channels[i].channel_cfg.differential) {
			val_mv = (int32_t)((int16_t)buf);
		} else {
			val_mv = (int32_t)buf;
		}
			
			// transform value
		sen_value = (val_mv/3.3);

			// printk("%"PRId32, val_mv);

		err = adc_raw_to_millivolts_dt(&adc_channels[i],
			&val_mv);

		/* conversion to mV may not be supported, skip if not */
		if (err < 0) {

			printk(" (value in mV not available)\n");

		} 
		else {

			// printk("Raw data: %"PRId32" (mV)\n", val_mv);
			//printk("M: %"PRId32"\n", sen_value);
			sprintf(soil, "%d", sen_value);//convert soil moisture value into string type

		}

	}
}

int main(void)
{
	char tx_buf[MSG_SIZE];
	//char output[MSG_SIZE];

	if (!device_is_ready(uart_dev))
	{
		printk("UART device not found!");
		return 0;
	}

	/* configure interrupt and callback to receive data */
	int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}
	uart_irq_rx_enable(uart_dev);


	// Initialize I2C bus
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0)); 
	svf_i2c_init();
	svf_adc_init();

	//Initialize a queue
	k_queue_init(&queue);



	/* indefinitely wait for input from the user */
	while(1){
		//read value from sensors and convert them into char data type
			svf_i2c_loop();
			svf_adc_loop();	

		//
		void *ret=k_queue_get(&queue, K_NO_WAIT);
		if (ret == &evt) {
			
				print_uart("high temp:");
				print_uart(temp);
				print_uart("\r\n");
			
		}
		
		if (k_msgq_get(&uart_msgq, &tx_buf, K_NO_WAIT) == 0 ) {

			print_uart(temp);
            print_uart("-");

			print_uart(hum);
			print_uart("-");

			print_uart(lightresult);
			print_uart("-");

			print_uart(soil);
			print_uart("\r\n");
			//k_event_wait(struct k_event *event, uint32_t events, bool reset, k_timeout_t timeout)¶		
		}

		k_msleep(300);
	
	}
	return 0;
}
