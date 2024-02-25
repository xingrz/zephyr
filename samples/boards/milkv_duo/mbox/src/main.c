/*
 * Copyright (c) 2024 honglin leng <a909204013@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#define TX_ID (1)
#define RX_ID (0)

#define QUEUE_DEPTH 10

struct k_msgq msgq;

static const struct device *mbox = DEVICE_DT_GET(DT_NODELABEL(mbox));
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

struct valid_t {
	unsigned char linux_valid;
	unsigned char rtos_valid;
};
typedef union resv_t {
	struct valid_t valid;
	unsigned short mstime;
} resv_t;

typedef struct cmdqu_t cmdqu_t;
/* cmdqu size should be 8 bytes because of mailbox buffer size */
struct cmdqu_t {
	unsigned char ip_id;
	unsigned char cmd_id: 7;
	unsigned char block: 1;
	union resv_t resv;
	unsigned int param_ptr;
} __packed;

enum SYS_CMD_ID {
	CMD_TEST_A = 0x10,
	CMD_TEST_B,
	CMD_TEST_C,
	CMD_DUO_LED,
	SYS_CMD_INFO_LIMIT,
};

enum DUO_LED_STATUS {
	DUO_LED_ON = 0x02,
	DUO_LED_OFF,
	DUO_LED_DONE,
};

cmdqu_t main_cmdq[QUEUE_DEPTH];

static void callback(const struct device *dev, uint32_t channel, void *user_data,
		     struct mbox_msg *data)
{
	cmdqu_t cmdq;

	memcpy(&cmdq, data->data, sizeof(cmdqu_t));
	k_msgq_put(&msgq, &cmdq, K_NO_WAIT);
}

int main(void)
{
	struct mbox_channel tx_channel;
	struct mbox_channel rx_channel;
	struct mbox_msg mbox_msg;
	int ret;
	cmdqu_t cmdq;

	k_msgq_init(&msgq, (char *)main_cmdq, sizeof(cmdqu_t), QUEUE_DEPTH);

	mbox_init_channel(&tx_channel, mbox, TX_ID);
	mbox_init_channel(&rx_channel, mbox, RX_ID);

	if (mbox_register_callback(&rx_channel, callback, NULL)) {
		printk("mbox_register_callback() error\n");
		return 0;
	}

	if (mbox_set_enabled(&rx_channel, 1)) {
		printk("mbox_set_enable() error\n");
		return 0;
	}

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	while (1) {
		ret = k_msgq_get(&msgq, &cmdq, K_FOREVER);
		if (ret == 0) {
			printk("cmdq->ip_id =%d\n", cmdq.ip_id);
			printk("cmdq->cmd_id =%d\n", cmdq.cmd_id);
			printk("cmdq->param_ptr =%x\n", cmdq.param_ptr);
			printk("Ping (on channel %d)\n", tx_channel.id);

			switch (cmdq.cmd_id) {
			case CMD_TEST_A:
				/* send to C906B */
				cmdq.cmd_id = CMD_TEST_A;
				cmdq.param_ptr = 0x12345678;
				cmdq.resv.valid.rtos_valid = 1;
				cmdq.resv.valid.linux_valid = 0;
				printk("recv cmd(%d) from C906B...send [0x%x] to C906B\n",
				       cmdq.cmd_id, cmdq.param_ptr);
				goto send_label;
			case CMD_TEST_B:
				printk("nothing to do...\n");
				break;
			case CMD_TEST_C:
				cmdq.cmd_id = CMD_TEST_C;
				cmdq.param_ptr = 0x55aa;
				cmdq.resv.valid.rtos_valid = 1;
				cmdq.resv.valid.linux_valid = 0;
				printk("recv cmd(%d) from C906B...send [0x%x] to C906B\n",
				       cmdq.cmd_id, cmdq.param_ptr);
				goto send_label;
			case CMD_DUO_LED:
				cmdq.cmd_id = CMD_DUO_LED;
				printk("recv cmd(%d) from C906B, param_ptr [0x%x]\n", cmdq.cmd_id,
				       cmdq.param_ptr);
				if (cmdq.param_ptr == DUO_LED_ON) {
					gpio_pin_set_dt(&led, 1);
				} else {
					gpio_pin_set_dt(&led, 0);
				}
				cmdq.param_ptr = DUO_LED_DONE;
				cmdq.resv.valid.rtos_valid = 1;
				cmdq.resv.valid.linux_valid = 0;
				printk("recv cmd(%d) from C906B...send [0x%x] to C906B\n",
				       cmdq.cmd_id, cmdq.param_ptr);
				goto send_label;
			default:
			send_label:
				mbox_msg.data = &cmdq;
				mbox_msg.size = sizeof(cmdqu_t);
				if (mbox_send(&tx_channel, &mbox_msg) < 0) {
					printk("mbox_send() error\n");
					return 0;
				}
				break;
			}
		}
	}
	return 0;
}
