
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


// NFC includes
#include <st25r3911b_nfca.h>
#include <nfc/ndef/msg_parser.h>
#include <nfc/t2t/parser.h>
#include <nfc/ndef/payload_type_common.h>
#include <nfc/ndef/text_rec.h>

#define NFCA_BD 128
#define BITS_IN_BYTE 8
#define MAX_TLV_BLOCKS 10
#define MAX_NDEF_RECORDS 10
#define NFCA_T2T_BUFFER_SIZE 1024
#define NFCA_LAST_BIT_MASK 0x80
#define NFCA_FDT_ALIGN_84 84
#define NFCA_FDT_ALIGN_20 20

#define NFC_T2T_READ_CMD 0x30
#define NFC_T2T_READ_CMD_LEN 0x02

#define NFC_T4T_ISODEP_FSD 256
#define NFC_T4T_ISODEP_RX_DATA_MAX_SIZE 1024
#define NFC_T4T_APDU_MAX_SIZE 1024

#define NFC_NDEF_REC_PARSER_BUFF_SIZE 128

#define NFC_TX_DATA_LEN NFC_T4T_ISODEP_FSD
#define NFC_RX_DATA_LEN NFC_T4T_ISODEP_FSD

#define T2T_MAX_DATA_EXCHANGE 16
#define TAG_TYPE_2_DATA_AREA_MULTIPLICATOR 8
#define TAG_TYPE_2_DATA_AREA_SIZE_OFFSET (NFC_T2T_CC_BLOCK_OFFSET + 2)
#define TAG_TYPE_2_BLOCKS_PER_EXCHANGE (T2T_MAX_DATA_EXCHANGE / NFC_T2T_BLOCK_SIZE)

#define TRANSMIT_DELAY 750
#define ALL_REQ_DELAY 500

static uint8_t tx_data[NFC_TX_DATA_LEN];
static uint8_t rx_data[NFC_RX_DATA_LEN];

static struct k_poll_event events[ST25R3911B_NFCA_EVENT_CNT];

static struct k_work_delayable transmit_work;
static struct k_work_delayable shutdown_field_work;
static struct k_work_delayable turn_on_work;


#define RING_BUF_SIZE 1024
uint8_t ring_buffer[RING_BUF_SIZE];

struct device *dev;
struct ring_buf ringbuf;

LOG_MODULE_REGISTER(nfc_led_ctl, LOG_LEVEL_INF);

static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));

#define SLEEP_MSEC 25U

#define STACKSIZE 2048
/* scheduling priority used by each thread */
#define LED_PRIORITY 7
#define NFC_PRIORITY 8

bool field_on = false;

int initNFC(void);
void handle_error(int err);

bool requesting_nfc_read;

enum states
{
	high_fade,
	on,
	off,
	low,
	low_fade,
	nfc_off,
	nfc_on
};

int program_state = off;

void processTxBuffer(const struct device *dev){
	if (uart_irq_tx_ready(dev))
		{
			uint8_t buffer[64];
			int rb_len, send_len;

			rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
			if (!rb_len)
			{
				LOG_DBG("Ring buffer empty, disable TX IRQ");
				uart_irq_tx_disable(dev);
				return;
			}

			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len)
			{
				LOG_ERR("Drop %d bytes", rb_len - send_len);
			}

			LOG_DBG("ringbuf -> tty fifo %d bytes", send_len);
		}
}

void writeToUart(const uint8_t *data, uint32_t size){
	ring_buf_put(&ringbuf, data, size);
}

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

			if (strchr(buffer, high_fade + '0') != NULL)
			{ // see if we have received a fade command '0'
				rb_len = ring_buf_put(&ringbuf, "high fade\n", 11);
				program_state = high_fade;
			}
			else if (strchr(buffer, on + '0') != NULL)
			{ // see if we have received an enable command '1'
				rb_len = ring_buf_put(&ringbuf, "on\n", 4);
				program_state = on;
			}
			else if (strchr(buffer, off + '0') != NULL)
			{ // see if we have received a disable command '2'
				rb_len = ring_buf_put(&ringbuf, "off\n", 5);
				program_state = off;
			}
			else if (strchr(buffer, low + '0') != NULL)
			{ // see if we have received a disable command '3'
				rb_len = ring_buf_put(&ringbuf, "low\n", 5);
				program_state = low;
			}
			else if (strchr(buffer, low_fade + '0') != NULL)
			{ // see if we have received a disable command '4'
				rb_len = ring_buf_put(&ringbuf, "low fade\n", 10);
				program_state = low_fade;
			}else if (strchr(buffer, nfc_off + '0') != NULL)
			{ // see if we have received a disable command '4'
				rb_len = ring_buf_put(&ringbuf, "nfc_off\n", 9);
				requesting_nfc_read = false;
				k_work_reschedule(&shutdown_field_work,K_MSEC(100));

			}else if (strchr(buffer, nfc_on + '0') != NULL)
			{ // see if we have received a disable command '4'
				rb_len = ring_buf_put(&ringbuf, "nfc_on\n", 8);
				requesting_nfc_read = true;
				k_work_reschedule(&turn_on_work, K_MSEC(100));
			}
			else if (strchr(buffer, 9 + '0') != NULL)
			{ // see if we have received a disable command '4'
				sys_reboot(0);
			}

			if (rb_len)
			{
				uart_irq_tx_enable(dev);
			}
		}
		processTxBuffer(dev);		
	}
}

static void setupUART()
{
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

void led_loop(void)
{
	int period = pwm_led0.period / 8U;
	int ret = pwm_set_dt(&pwm_led0, period, period);

	int max_pulse_width = period - (period / 12);
	int min_pulse_width = 0;

	double scale = 0;
	uint32_t pulse_width = 0;

	while (1)
	{
		switch (program_state)
		{
		case high_fade:
			scale = (cos(8.0 * time_sec) + 1.0) / 2.0;
			pulse_width = (uint32_t)((max_pulse_width - min_pulse_width) * scale) + min_pulse_width;
			break;

		case on:
			pulse_width = 0U;
			break;

		case off:
			pulse_width = period;
			break;

		case low:
			pulse_width = max_pulse_width;
			break;

		case low_fade:
			scale = (cos(4.0 * time_sec) + 1.0) / 2.0;
			pulse_width = (uint32_t)((period - max_pulse_width) * scale) + max_pulse_width;
			break;

		default:
			break;
		}

		ret = pwm_set_dt(&pwm_led0, period, pulse_width);
		if (ret)
		{
			LOG_INF("Error %d: failed to set pulse width\n", ret);
			return;
		}

		k_msleep(SLEEP_MSEC);
		time_sec += (double)SLEEP_MSEC / (double)MSEC_PER_SEC;
	}
}



static struct st25r3911b_nfca_buf tx_buf = {
	.data = tx_data,
	.len = sizeof(tx_data)};

static const struct st25r3911b_nfca_buf rx_buf = {
	.data = rx_data,
	.len = sizeof(rx_data)};

enum nfc_tag_type
{
	NFC_TAG_TYPE_UNSUPPORTED = 0,
	NFC_TAG_TYPE_T2T,
	NFC_TAG_TYPE_T4T
};

enum t2t_state
{
	T2T_IDLE,
	T2T_HEADER_READ,
	T2T_DATA_READ
};

struct t2t_tag
{
	enum t2t_state state;
	uint16_t data_bytes;
	uint8_t data[NFCA_T2T_BUFFER_SIZE];
};


static enum nfc_tag_type tag_type;
static struct t2t_tag t2t;

static void nfc_tag_detect(bool all_request)
{
	int err;
	enum st25r3911b_nfca_detect_cmd cmd;

	tag_type = NFC_TAG_TYPE_UNSUPPORTED;

	cmd = all_request ? ST25R3911B_NFCA_DETECT_CMD_ALL_REQ : ST25R3911B_NFCA_DETECT_CMD_SENS_REQ;

	err = st25r3911b_nfca_tag_detect(cmd);
	if (err)
	{
		printk("Tag detect error: %d.\n", err);
	}
}

static int ftd_calculate(uint8_t *data, size_t len)
{
	uint8_t ftd_align;

	ftd_align = (data[len - 1] & NFCA_LAST_BIT_MASK) ? NFCA_FDT_ALIGN_84 : NFCA_FDT_ALIGN_20;

	return len * NFCA_BD * BITS_IN_BYTE + ftd_align;
}

static int nfc_t2t_read_block_cmd_make(uint8_t *tx_data,
									   size_t tx_data_size,
									   uint8_t block_num)
{
	if (!tx_data)
	{
		return -EINVAL;
	}

	if (tx_data_size < NFC_T2T_READ_CMD_LEN)
	{
		return -ENOMEM;
	}

	tx_data[0] = NFC_T2T_READ_CMD;
	tx_data[1] = block_num;

	return 0;
}

static int t2t_header_read(void)
{
	int err;
	int ftd;

	err = nfc_t2t_read_block_cmd_make(tx_data, sizeof(tx_data), 0);
	if (err)
	{
		return err;
	}

	tx_buf.data = tx_data;
	tx_buf.len = NFC_T2T_READ_CMD_LEN;
	
	ftd = ftd_calculate(tx_data, NFC_T2T_READ_CMD_LEN);

	t2t.state = T2T_HEADER_READ;
	
	err = st25r3911b_nfca_transfer_with_crc(&tx_buf, &rx_buf, ftd);

	return err;
}

static void ndef_data_analyze(const uint8_t *ndef_msg_buff, size_t nfc_data_len)
{
	int err;
	struct nfc_ndef_msg_desc *ndef_msg_desc;
	uint8_t desc_buf[NFC_NDEF_PARSER_REQUIRED_MEM(MAX_NDEF_RECORDS)];
	size_t desc_buf_len = sizeof(desc_buf);

	err = nfc_ndef_msg_parse(desc_buf,
							 &desc_buf_len,
							 ndef_msg_buff,
							 &nfc_data_len);
	if (err)
	{
		printk("Error during parsing a NDEF message, err: %d.\n", err);
		return;
	}

	ndef_msg_desc = (struct nfc_ndef_msg_desc *)desc_buf;


	for (int i = 0; i < ndef_msg_desc->record_count; i++)
	{
		struct nfc_ndef_record_desc *record = ndef_msg_desc->record[i];

		struct nfc_ndef_text_rec_payload text_rec;

		if (record->payload_constructor == (payload_constructor_t)nfc_ndef_bin_payload_memcopy)
		{
			const struct nfc_ndef_bin_payload_desc *bin_pay_desc = record->payload_descriptor;

			if (bin_pay_desc->payload != NULL)
			{
				writeToUart("nfc: ",5);
				writeToUart((uint8_t *)bin_pay_desc->payload,bin_pay_desc->payload_length);
				writeToUart("\n",1);
				uart_irq_tx_enable(dev);
				processTxBuffer(dev);
				requesting_nfc_read = false;
			}
			else
			{
				LOG_INF("No payload");
			}
		}
	}
}

static void t2t_data_read_complete(uint8_t *data)
{
	int err;
	printk("data read complete\n");
	if (!data)
	{
		printk("No T2T data read.\n");
		return;
	}

	/* Declaration of Type 2 Tag structure. */
	NFC_T2T_DESC_DEF(tag_data, MAX_TLV_BLOCKS);
	struct nfc_t2t *t2t = &NFC_T2T_DESC(tag_data);

	err = nfc_t2t_parse(t2t, data);
	if (err)
	{
		printk("error: %d, Not enough memory to read whole tag. Printing what have been read.\n",err);
		handle_error(err);
		return;
	}

	// nfc_t2t_printout(t2t);

	struct nfc_t2t_tlv_block *tlv_block = t2t->tlv_block_array;

	for (size_t i = 0; i < t2t->tlv_count; i++)
	{
		if (tlv_block->tag == NFC_T2T_TLV_NDEF_MESSAGE)
		{
			ndef_data_analyze(tlv_block->value, tlv_block->length);
			tlv_block++;
		}
	}

	// shut down the field
	k_work_reschedule(&shutdown_field_work, K_MSEC(TRANSMIT_DELAY));
}

static int t2t_on_data_read(const uint8_t *data, size_t data_len,
							void (*t2t_read_complete)(uint8_t *))
{
	int err;
	int ftd;
	uint8_t block_to_read;
	uint16_t offset = 0;
	static uint8_t block_num;

	block_to_read = t2t.data_bytes / NFC_T2T_BLOCK_SIZE;
	offset = block_num * NFC_T2T_BLOCK_SIZE;

	memcpy(t2t.data + offset, data, data_len);

	block_num += TAG_TYPE_2_BLOCKS_PER_EXCHANGE;

	if (block_num > block_to_read)
	{
		block_num = 0;
		t2t.state = T2T_IDLE;

		if (t2t_read_complete)
		{
			t2t_read_complete(t2t.data);
		}

		return 0;
	}

	err = nfc_t2t_read_block_cmd_make(tx_data, sizeof(tx_data), block_num);
	if (err)
	{
		return err;
	}

	tx_buf.data = tx_data;
	tx_buf.len = NFC_T2T_READ_CMD_LEN;

	ftd = ftd_calculate(tx_data, NFC_T2T_READ_CMD_LEN);

	err = st25r3911b_nfca_transfer_with_crc(&tx_buf, &rx_buf, ftd);

	return err;
}

static int on_t2t_transfer_complete(const uint8_t *data, size_t len)
{
	switch (t2t.state)
	{
	case T2T_HEADER_READ:
		t2t.data_bytes = TAG_TYPE_2_DATA_AREA_MULTIPLICATOR *
						 data[TAG_TYPE_2_DATA_AREA_SIZE_OFFSET];

		if ((t2t.data_bytes + NFC_T2T_FIRST_DATA_BLOCK_OFFSET) > sizeof(t2t.data))
		{
			return -ENOMEM;
		}

		t2t.state = T2T_DATA_READ;

		return t2t_on_data_read(data, len, t2t_data_read_complete);

	case T2T_DATA_READ:
		return t2t_on_data_read(data, len, t2t_data_read_complete);

	default:
		return -EFAULT;
	}
}


static void nfc_field_on(void)
{
	nfc_tag_detect(true);
	printk("NFC field on.\n");
	field_on = true;
}

static void nfc_timeout(bool tag_sleep)
{
	printk("NFC timeout\n");
	k_sleep(K_MSEC(ALL_REQ_DELAY));
	handle_error(-89); 
	// /* Sleep will block processing loop. Accepted as it is short. */
	// k_sleep(K_MSEC(ALL_REQ_DELAY));

	// nfc_tag_detect(false);
}

static void nfc_field_off(void)
{
	printk("NFC field off.\n");
	field_on = false;
}

static void nfc_field_off_handler(struct k_work *work){
	st25r3911b_nfca_field_off();
}

static void nfc_field_on_handler(struct k_work *work){
	if (requesting_nfc_read && !field_on) {
		printk("calling field on\n");
		st25r3911b_nfca_field_on();
	}else{

		printk("requesting tag detect\n");
		nfc_tag_detect(false);
	}
}

static void tag_detected(const struct st25r3911b_nfca_sens_resp *sens_resp)
{
	printk("Anticollision: 0x%x Platform: 0x%x.\n",
		   sens_resp->anticollison,
		   sens_resp->platform_info);

	int err = st25r3911b_nfca_anticollision_start();

	if (err)
	{
		printk("Anticollision error: %d.\n", err);
	}
}

static void anticollision_completed(const struct st25r3911b_nfca_tag_info *tag_info,
									int err)
{
	if (err)
	{
		printk("Error during anticollision avoidance.\n");

		// nfc_tag_detect(false);
		return;
	}

	printk("Tag info, type: %d.\n", tag_info->type);

	if (tag_info->type == ST25R3911B_NFCA_TAG_TYPE_T2T)
	{
		printk("Type 2 Tag.\n");

		tag_type = NFC_TAG_TYPE_T2T;

		err = t2t_header_read();
		if (err)
		{
			printk("Type 2 Tag data read error %d.\n", err);
			handle_error(-93);
		}
	}
}

static void transfer_completed(const uint8_t *data, size_t len, int err)
{
	if (err)
	{
		printk("NFC Transfer error: %d.\n", err);
		handle_error(-91);
		return;
	}

	switch (tag_type)
	{
	case NFC_TAG_TYPE_T2T:
		err = on_t2t_transfer_complete(data, len);
		if (err)
		{
			printk("NFC-A T2T read error: %d.\n", err);
			handle_error(-92);
		}
		// LOG_HEXDUMP_INF(data, len, "data read");

		break;

	default:
		break;
	}
}

static void tag_sleep(void)
{
	printk("Tag entered the Sleep state.\n");
}

static const struct st25r3911b_nfca_cb cb = {
	.field_on = nfc_field_on,
	.field_off = nfc_field_off,
	.tag_detected = tag_detected,
	.anticollision_completed = anticollision_completed,
	.rx_timeout = nfc_timeout,
	.transfer_completed = transfer_completed,
	.tag_sleep = tag_sleep};


int initNFC(void){

	int err = st25r3911b_nfca_init(events, ARRAY_SIZE(events), &cb);
	if (err)
	{
		LOG_INF("NFCA initialization failed err: %d.\n", err);
		return err;
	}
	return 1;
}

void handle_error(int err){
	// printk("Error received %d\n",err);
	bool reschedule = false;

	switch (err)
	{
	case -1:
		// General process error
		// this seems to be followed by a timeout
		reschedule = false; 
		break;
	case -13:
		// RF Collision
		reschedule = true;
		break;
	case -22:
		// Ran out of memory??
		reschedule = true;
		break;

	case -89:
		// nfc timeout
		reschedule = true;
		break;

	case -91:
		// transfer error
		reschedule = true;
		break;

	case -92:
		// tag read error
		reschedule = true;
		break;

	case -93:
		// tag header read error
		reschedule = true;
		break;
	
	default:
		break;
	}

	if( reschedule ){
		// writeToUart("nfc error\n", 11);
		// uart_irq_tx_enable(dev);
		// processTxBuffer(dev);
		requesting_nfc_read = true;
		k_work_reschedule(&turn_on_work, K_MSEC(100));
	}

	// nfc_tag_detect(false);
	// k_work_reschedule(&shutdown_field_work, K_MSEC(100)); // reschedule turn off
	// k_work_reschedule(&turn_on_work, K_MSEC(500)); // reschedule turn on
}


void nfc_loop(void){


	printk("starting nfc loop\n");
	while (initNFC() != 1){
		LOG_INF("unable to init NFC\n");
		k_busy_wait(1000000); //wait for 1sec and try again
	}

	int err;
	while (true)
	{
		k_poll(events, ARRAY_SIZE(events), K_FOREVER);
		// printk("processing state\n");
		err = st25r3911b_nfca_process();
		if (err)
		{
			// printk("NFC-A process failed, err: %d.\n", err);
			handle_error(err);
		}
	}
}


K_THREAD_DEFINE(led_loop_id, STACKSIZE, led_loop, NULL, NULL, NULL,
				LED_PRIORITY, 0, 0);


K_THREAD_DEFINE(nfc_loop_id, STACKSIZE, nfc_loop, NULL, NULL, NULL,
				NFC_PRIORITY, 0, 2000);

void main(void)
{
	

	k_work_init_delayable(&turn_on_work, nfc_field_on_handler);
	k_work_init_delayable(&shutdown_field_work, nfc_field_off_handler);
	
	if (!device_is_ready(pwm_led0.dev))
	{
		LOG_INF("Error: PWM device %s is not ready\n",
				pwm_led0.dev->name);
		return;
	}

	setupUART();

	while (1)
	{
		k_msleep(1000);
		
	}
	
}