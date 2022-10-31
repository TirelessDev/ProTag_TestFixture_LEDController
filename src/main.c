
#include <stdio.h>
#include <string.h>
#include <device.h>
#include <drivers/uart.h>
#include <zephyr.h>
#include <sys/ring_buffer.h>

#include <usb/usb_device.h>
#include <logging/log.h>

#include <device.h>
#include <drivers/pwm.h>
#include <math.h>

static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));

#define NUM_STEPS 400U
#define SLEEP_MSEC 25U

enum states
{
	fade,
	enable,
	disable
};

int program_state = fade;

LOG_MODULE_REGISTER(cdc_acm_echo, LOG_LEVEL_INF);

#define RING_BUF_SIZE 1024
uint8_t ring_buffer[RING_BUF_SIZE];

struct ring_buf ringbuf;

static void interrupt_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev))
	{
		if (uart_irq_rx_ready(dev))
		{
			int recv_len, rb_len;
			uint8_t buffer[64];
			size_t len = MIN(ring_buf_space_get(&ringbuf),
							 sizeof(buffer));

			recv_len = uart_fifo_read(dev, buffer, len);
			if (recv_len < 0)
			{
				LOG_ERR("Failed to read UART FIFO");
				recv_len = 0;
			};

			if (strchr(buffer, '0') != NULL)
			{ // see if we have received a fade command '0'
				rb_len = ring_buf_put(&ringbuf, "fading\n", 8);
				program_state = fade;
			}
			else if (strchr(buffer, '1') != NULL)
			{ // see if we have received an enable command '1'
				rb_len = ring_buf_put(&ringbuf, "enable\n", 8);
				program_state = enable;
			}
			else if (strchr(buffer, '2') != NULL)
			{ // see if we have received a disable command '2'
				rb_len = ring_buf_put(&ringbuf, "disable\n", 9);
				program_state = disable;
			}

			// rb_len = ring_buf_put(&ringbuf, buffer, recv_len);
			// if (rb_len < recv_len) {
			// 	LOG_ERR("Drop %u bytes", recv_len - rb_len);
			// }

			LOG_DBG("tty fifo -> ringbuf %d bytes", rb_len);
			if (rb_len)
			{
				uart_irq_tx_enable(dev);
			}
		}

		if (uart_irq_tx_ready(dev))
		{
			uint8_t buffer[64];
			int rb_len, send_len;

			rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
			if (!rb_len)
			{
				LOG_DBG("Ring buffer empty, disable TX IRQ");
				uart_irq_tx_disable(dev);
				continue;
			}

			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len)
			{
				LOG_ERR("Drop %d bytes", rb_len - send_len);
			}

			LOG_DBG("ringbuf -> tty fifo %d bytes", send_len);
		}
	}
}

static void setupUART()
{
	const struct device *dev;
	uint32_t baudrate, dtr = 0U;
	int ret;

	dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (!device_is_ready(dev))
	{
		LOG_ERR("CDC ACM device not ready");
		return;
	}

	ret = usb_enable(NULL);
	if (ret != 0)
	{
		LOG_ERR("Failed to enable USB");
		return;
	}

	ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);

	// LOG_INF("Wait for DTR");

	// while (true)
	// {
	// 	uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
	// 	if (dtr)
	// 	{
	// 		break;
	// 	}
	// 	else
	// 	{
	// 		/* Give CPU resources to low priority threads. */
	// 		k_sleep(K_MSEC(100));
	// 	}
	// }

	// LOG_INF("DTR set");

	// /* They are optional, we use them to test the interrupt endpoint */
	// ret = uart_line_ctrl_set(dev, UART_LINE_CTRL_DCD, 1);
	// if (ret)
	// {
	// 	LOG_WRN("Failed to set DCD, ret code %d", ret);
	// }

	// ret = uart_line_ctrl_set(dev, UART_LINE_CTRL_DSR, 1);
	// if (ret)
	// {
	// 	LOG_WRN("Failed to set DSR, ret code %d", ret);
	// }

	/* Wait 1 sec for the host to do all settings */
	k_busy_wait(1000000);

	ret = uart_line_ctrl_get(dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret)
	{
		LOG_WRN("Failed to get baudrate, ret code %d", ret);
	}
	else
	{
		LOG_INF("Baudrate detected: %d", baudrate);
	}

	uart_irq_callback_set(dev, interrupt_handler);

	/* Enable rx interrupts */
	uart_irq_rx_enable(dev);
}

double time_sec = 0;

void main(void)
{
	setupUART();

	uint32_t pulse_width = 0U;
	uint32_t step = pwm_led0.period / NUM_STEPS;
	uint8_t dir = 1U;
	int ret;

	printk("PWM-based LED fade\n");

	if (!device_is_ready(pwm_led0.dev))
	{
		printk("Error: PWM device %s is not ready\n",
			   pwm_led0.dev->name);
		return;
	}

	int period = pwm_led0.period / 8U;
	int max_pulse_width = period - period/5;
	int min_pulse_width = step;
double scale;
	while (1)
	{

		ret = pwm_set_dt(&pwm_led0, period, pulse_width);
		if (ret)
		{
			printk("Error %d: failed to set pulse width\n", ret);
			return;
		}

		switch (program_state)
		{
		case fade:
			scale = (cos(2.0*time_sec)+1.0)/2.0;
			pulse_width =  (uint32_t)((max_pulse_width-min_pulse_width)*scale)+min_pulse_width;
			break;

		case enable:
			pulse_width = 0U;
			break;

		case disable:
			pulse_width = period;
			break;

		default:
			break;
		}

		k_sleep(K_MSEC(SLEEP_MSEC));
		time_sec+= (double)SLEEP_MSEC/(double)MSEC_PER_SEC;
	}
}
