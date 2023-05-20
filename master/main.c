
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <string.h>

  #define UART_DEVICE_NODE DT_ALIAS(uart1)
  static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* change this to any other UART peripheral if desired */

/*
 * Copyright (c) 2022 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */



/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE0 DT_CHOSEN(zephyr_shell_uart)
static const struct device *const uart_dev0 = DEVICE_DT_GET(UART_DEVICE_NODE0);

#define MSG_SIZE 32

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);
K_MSGQ_DEFINE(uart_msgq0, MSG_SIZE, 10, 4);



/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
//between terminal and master board
void serial_cb0(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev0)) {
		return;
	}

	while (uart_irq_rx_ready(uart_dev0)) {

		uart_fifo_read(uart_dev0, &c, 1);

		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			// terminate string 
			rx_buf[rx_buf_pos] = '\0';

			// if queue is full, message is silently dropped 
			k_msgq_put(&uart_msgq0, &rx_buf, K_NO_WAIT);

			// reset the buffer (it was copied to the msgq) 
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		// else: characters beyond buffer size are dropped 
	}
}
//between sensor and master board
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

void main(void)
{
	char tx_buf[MSG_SIZE];
	char output[MSG_SIZE];
	

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return;
	}{
		printk("UART connected\n");
	}

	/* configure interrupt and callback to receive data */
	int ret = uart_irq_callback_user_data_set(uart_dev0, serial_cb0, NULL);
	int ret1=uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

	if (ret1<0 || ret<0) {
		if (ret1 == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret1 == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret1);
		}
		return;
	}else{
		printk("configure interrupt done\n");
	}
	uart_irq_rx_enable(uart_dev0);
	uart_irq_rx_enable(uart_dev);
	printk("units temperature Celsius, light lux\n");
	printk("light should be between 10,000 and 25,000\n");
/*	# 0  ~300     dry soil
	# 300~700     humid soil
	# 700~950     in water
*/
	printk("soilmoisture between 1 and 300 means dry soil\n");
	printk("soilmoisture between 300 and 700 means dry soil\n");
	printk("soilmoisture between 700 and 950 means dry soil\n");

		/* indefinitely wait for input from the user */
			
	while(1){
				if(k_msgq_get(&uart_msgq0, &tx_buf, K_NO_WAIT) == 0){
				
				 print_uart(tx_buf);
            	 print_uart("\r\n");

					/* Wait for the "sensor" message from the sensor board */
					if (k_msgq_get(&uart_msgq, &output, K_FOREVER) == 0) {
						printk("temp-humidity-light-soilmoisture: %s\r\n", output);
					}
					//k_msgq_purge(&uart_msgq);//Purge a message queue.
	
				}

				//if temp-humidity-light-soilmoisture is received by the first k_msgq_get(), then do not receive it again in the next k_msgq_get().
				
					if (k_msgq_get(&uart_msgq, &output, K_NO_WAIT) == 0 && output[1]=='h') {
								printk("warning: %s\r\n", output);
								
					}
				
				
				
				
				
			
	}
 
	
}
