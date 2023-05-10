#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <string.h>


#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <bme680_reg.h>
#include <veml7700_reg.h>

#define BME680_ADDR     0x77
#define veml7700        0x10
#define COMMAND_CODE    0x00
#define MY_STACK_SIZE   5000

const struct device *i2c_dev ;
uint8_t  data[3];//temp
int32_t p1,p2,p3;//temp

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 32

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/*event queue*/
struct k_queue evt_queue;

typedef enum _event_t 
{
    b1_evt = 0,
    b2_evt = 1,
    b3_evt = 2,
    no_evt = 3
} event_t;

///////
int32_t temp_convert(uint32_t temp_adc, int32_t  p1,int32_t  p2,int32_t  p3 ){
    int32_t var1 = ((int32_t)temp_adc >> 3) - ((int32_t)p1<< 1);
    int32_t var2 = (var1* (int32_t)p2) >> 11;
    int32_t var3 = ((((var1 >> 1) * (var1 >> 1)) >> 12) * ((int32_t)p3 << 4)) >> 14;
    int32_t t_fine = var2 + var3;
    int32_t temp_comp = (int32_t)((t_fine * 5) + 128) >> 8;
    return temp_comp;
}

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
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

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}
void ini(){
    //uint8_t  data[3];
   // uint8_t h_data[2];//humidity register
    //uint32_t temp_adc, humidity_adc;
    uint8_t  par_t[5];
    // uint8_t par_h[9];//humidity register
    // int32_t p1,p2,p3;
    // int32_t h1,h2,h3,h4,h5,h6,h7;
    // int32_t temp_comp;
    // uint8_t light[2];
    // Initialize I2C bus
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0)); 

    //tempurature register
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,0xE9,&par_t[0]);
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,0XEA,&par_t[1]);
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,0x8A,&par_t[2]);
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,0x8B,&par_t[3]);
    i2c_reg_read_byte(i2c_dev, BME680_ADDR,0x8c,&par_t[4]);

    p1=(int32_t) ((uint16_t)par_t[0] | ((uint16_t)par_t[1])<<8);
    p2=(int32_t)((uint16_t)par_t[2] | ((uint16_t)par_t[3])<<8);
    p3=(int32_t)par_t[4];

}
int readTemp(){
    i2c_reg_write_byte(i2c_dev, BME680_ADDR,BME680_CTRL_MEAS,0b010 << 5 | 0b01);//force mode: 01

    i2c_reg_read_byte(i2c_dev, BME680_ADDR,BME680_TEMP_MSB,&data[0]);
        i2c_reg_read_byte(i2c_dev, BME680_ADDR,BME680_TEMP_LSB,&data[1]);
        i2c_reg_read_byte(i2c_dev, BME680_ADDR,BME680_TEMP_XLSB,&data[2]); 

     uint32_t   temp_adc=((uint32_t)data[0])<<12 | ((uint32_t)data[1])<<4 | ((uint32_t)data[2])>>4;
     int32_t   temp_comp=temp_convert(temp_adc,p1,p2,p3);
     event_t evt;
     if((temp_comp/100)>30){
        evt=b1_evt;
        int err=k_queue_alloc_append(&evt_queue,&evt);
        if(err==0){
            printk("put event in the queue");//0 – on success;
        } else if(err==-ENOMEM){
            printk("Error: insufficient memory in the caller's resource pool");
            }// -ENOMEM – if there isn’t sufficient RAM in the caller’s resource pool
        else{
            printk("Error: unable to append event to the queue");
        }
     }
        
        return temp_comp/100;

        
        
}


void main(void)
{
	char tx_buf[MSG_SIZE];

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return;
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
		return;
	}
	uart_irq_rx_enable(uart_dev);

    /* Event queue setup */
    k_queue_init(&evt_queue);
    if(k_queue_is_empty(&evt_queue)==0){
        print_uart("data is available.");
    }else{
        print_uart("queue is empty");
    }
    event_t evt = no_evt;

	print_uart("Hello! I'm your echo bot.\r\n");
	print_uart("Tell me something and press enter:\r\n");
    //initial temp
    ini();
    
    char c[20];

	/* indefinitely wait for input from the user */
	while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0  ) {
		print_uart("Echo: ");
		print_uart(tx_buf);
        print_uart("--temp:");
        
        sprintf(c, "%d", readTemp());
        print_uart(c);

        if (k_queue_remove(&evt_queue, &evt))
            { 
                //return evt; 
                //if(evt==b1_evt){
                    print_uart("too high temp");
                //}
            }
            
        
        
        print_uart("\r\n");

	}
}

