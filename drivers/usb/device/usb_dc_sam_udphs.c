/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_sam_udphs

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/usb/usb_dc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb_dc_sam_udphs, CONFIG_USB_DRIVER_LOG_LEVEL);

#include <soc.h>

#define SHIFT_DMA      24 /* Bits that should be shifted to access DMA control bits. */
#define SHIFT_INTERUPT 8  /* Bits that should be shifted to access interrupt bits. */

#define DMA_MAX_FIFO_SIZE 65536 /* Max size of the FMA FIFO */

#define EPT_VIRTUAL_SIZE 16384

enum usb_device_ep_state {
	UDP_ENDPOINT_DISABLED,	/* Endpoint is disabled */
	UDP_ENDPOINT_HALTED,	/* Endpoint is halted (i.e. STALLs every request) */
	UDP_ENDPOINT_IDLE,	/* Endpoint is idle (i.e. ready for transmission) */
	UDP_ENDPOINT_SENDING,	/* Endpoint is sending data */
	UDP_ENDPOINT_RECEIVING, /* Endpoint is receiving data */
};

struct usb_device_transfer {
	char *data;
	volatile int buffered;
	volatile int transferred;
	volatile int remaining;
};

struct usb_device_ep_data {
	volatile enum usb_device_ep_state state;
	uint8_t bank;
	uint16_t mps;
	struct usb_device_transfer transfer;
	uint8_t send_zlp;
	usb_dc_ep_callback cb_in;
	usb_dc_ep_callback cb_out;
};

struct usb_device_data {
	usb_dc_status_callback status_cb;
	struct usb_device_ep_data ep_data[UDPHSEPT_NUMBER];
};

static struct usb_device_data dev_data;

static void usb_dc_enable_clock(void)
{
	/* Disable write protect */
	PMC->PMC_WPMR = PMC_WPMR_WPKEY_PASSWD;

	/* Start the USB PLL */
	PMC->CKGR_UCKR = CKGR_UCKR_UPLLCOUNT(15) | CKGR_UCKR_UPLLEN;

	/* Wait for it to be ready */
	while (!(PMC->PMC_SR & PMC_SR_LOCKU)) {
		k_yield();
	}

	/* Enable write protect */
	PMC->PMC_WPMR = PMC_WPMR_WPKEY_PASSWD | PMC_WPMR_WPEN;
}

static void usb_dc_disable_clock(void)
{
	/* Disable the USB PLL */
	PMC->CKGR_UCKR &= ~CKGR_UCKR_UPLLEN;
}

static void usb_dc_end_of_transfer(uint8_t ep_idx)
{
	struct usb_device_ep_data *ep = &dev_data.ep_data[ep_idx];
	struct usb_device_transfer *transfer = &ep->transfer;

	if (ep->state == UDP_ENDPOINT_RECEIVING || ep->state == UDP_ENDPOINT_SENDING) {
		if (ep->state == UDP_ENDPOINT_SENDING) {
			ep->send_zlp = 0;
		}

		ep->state = UDP_ENDPOINT_IDLE;
	}
}

static void usb_dc_write_payload(uint8_t ep_idx)
{
	struct usb_device_ep_data *ep = &dev_data.ep_data[ep_idx];
	struct usb_device_transfer *transfer = &ep->transfer;

	uint8_t *fifo = (uint8_t *)((uint32_t *)UDPHS_RAM_ADDR + EPT_VIRTUAL_SIZE * ep_idx);

	int size = MIN(ep->mps, transfer->remaining);

	transfer->buffered += size;
	transfer->remaining -= size;

	int count = 0;
	while (size > 0) {
		fifo[count] = *(transfer->data);
		transfer->data++;
		size--;
		count++;
	}
}

static void usb_dc_read_payload(uint8_t ep_idx, int size)
{

	struct usb_device_ep_data *ep = &dev_data.ep_data[ep_idx];
	struct usb_device_transfer *transfer = &ep->transfer;

	uint8_t *fifo = (uint8_t *)((uint32_t *)UDPHS_RAM_ADDR + EPT_VIRTUAL_SIZE * ep_idx);

	if (size > transfer->remaining) {
		transfer->buffered += size - transfer->remaining;
		size = transfer->remaining;
	}

	transfer->remaining -= size;
	transfer->transferred += size;

	int count = 0;
	while (size > 0) {
		*transfer->data = fifo[count];
		transfer->data++;
		size--;
		count++;
	}
}

static void usb_dc_reset_endpoints(void)
{
	struct usb_device_ep_data *ep;
	struct usb_device_transfer *transfer;

	for (uint8_t ep_idx = 0; ep_idx < UDPHSEPT_NUMBER; ep_idx++) {
		ep = &dev_data.ep_data[ep_idx];
		transfer = &ep->transfer;

		transfer->data = 0;
		transfer->transferred = -1;
		transfer->buffered = -1;
		transfer->remaining = -1;

		ep->bank = 0;
		ep->state = UDP_ENDPOINT_DISABLED;
		ep->send_zlp = 0;
	}
}

static void usb_dc_disable_endpoints(void)
{
	for (uint8_t ep_idx = 1; ep_idx < UDPHSEPT_NUMBER; ep_idx++) {
		usb_dc_end_of_transfer(ep_idx);
		dev_data.ep_data[ep_idx].state = UDP_ENDPOINT_DISABLED;
	}
}

static void usb_dc_ep_isr(uint8_t ep_idx)
{
	struct usb_device_ep_data *ep = &dev_data.ep_data[ep_idx];
	struct usb_device_transfer *transfer = &ep->transfer;

	uint32_t eptsta = UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTSTA;
	LOG_DBG("ep_idx: %d, eptsta: %08x", ep_idx, eptsta);

	/* IN packet sent */
	if ((UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTCTL & UDPHS_EPTCTL_TXRDY) &&
	    !(eptsta & UDPHS_EPTSTA_TXRDY)) {
		LOG_DBG("ep_idx: %d, TXRDY", ep_idx);

		if (ep->state == UDP_ENDPOINT_SENDING) {
			if (transfer->buffered > 0) {
				transfer->transferred += transfer->buffered;
				transfer->buffered = 0;
			}

			if (transfer->buffered == 0 && transfer->transferred == 0 &&
			    transfer->remaining == 0 && ep->send_zlp == 0) {
				ep->send_zlp = 1;
			}

			if (transfer->remaining > 0 || ep->send_zlp == 1) {
				ep->send_zlp = 2;

				/* Send next packet */
				usb_dc_write_payload(ep_idx);
				UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTSETSTA = UDPHS_EPTSETSTA_TXRDY;
			} else {
				/* Disable interrupt if this is not a control endpoint */
				if (!(UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTCFG &
				      UDPHS_EPTCFG_EPT_TYPE_CTRL8)) {
					UDPHS->UDPHS_IEN &= ~(BIT(ep_idx) << SHIFT_INTERUPT);
				}

				UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTCTLDIS = UDPHS_EPTCTLDIS_TXRDY;

				usb_dc_end_of_transfer(ep_idx);
				ep->send_zlp = 0;
			}
		}
	}

	/* OUT packet received */
	if (eptsta & UDPHS_EPTSTA_RXRDY_TXKL) {
		LOG_DBG("ep_idx: %d, RXRDY_TXKL", ep_idx);

		if (ep->state == UDP_ENDPOINT_RECEIVING) {
			/* Retrieve data and store it into the current transfer buffer */
			uint32_t size = (eptsta & UDPHS_EPTSTA_BYTE_COUNT_Msk) >>
					UDPHS_EPTSTA_BYTE_COUNT_Pos;
			usb_dc_read_payload(ep_idx, size);

			UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTCLRSTA = UDPHS_EPTCLRSTA_RXRDY_TXKL;

			/* Check if the transfer is finished */
			if (transfer->remaining == 0 || size < ep->mps) {
				UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTCTLDIS =
					UDPHS_EPTCTLDIS_RXRDY_TXKL;

				/* Disable interrupt if this is not a control endpoint */
				if (!(UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTCFG &
				      UDPHS_EPTCFG_EPT_TYPE_CTRL8)) {
					UDPHS->UDPHS_IEN &= ~(BIT(ep_idx) << SHIFT_INTERUPT);
				}

				usb_dc_end_of_transfer(ep_idx);
			}
		} else {
		}
	}

	/* STALL sent */
	if (eptsta & UDPHS_EPTSTA_STALL_SNT) {
		LOG_DBG("ep_idx: %d, STALL", ep_idx);
		UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTCLRSTA = UDPHS_EPTCLRSTA_STALL_SNT;
	}

	/* SETUP packet received */
	if (eptsta & UDPHS_EPTSTA_RX_SETUP) {
		LOG_DBG("ep_idx: %d, SETUP", ep_idx);
		usb_dc_ep_fifo_reset(ep_idx);
		UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTCLRSTA = UDPHS_EPTCLRSTA_RX_SETUP;
		ep->cb_out(ep_idx | USB_EP_DIR_OUT, USB_DC_EP_SETUP);
	}

	/* Data Packet Sent Interrupt */
	if (eptsta & UDPHS_EPTSTA_TX_COMPLT) {
		LOG_DBG("ep_idx: %d, EP_DATA_IN", ep_idx);
		UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTCLRSTA = UDPHS_EPTCLRSTA_TX_COMPLT;
		usb_dc_ep_fifo_reset(ep_idx);
		ep->cb_in(ep_idx | USB_EP_DIR_IN, USB_DC_EP_DATA_IN);
	}
}

static void usb_dc_isr(void)
{
	irq_disable(DT_INST_IRQN(0));

	uint32_t intsta = UDPHS->UDPHS_INTSTA & UDPHS->UDPHS_IEN;
	LOG_DBG("intsta: %08x", intsta);

	while (intsta != 0) {
#ifdef CONFIG_USB_DEVICE_SOF
		/* tart Of Frame (SOF) */
		if (intsta & UDPHS_INTSTA_INT_SOF) {
			dev_data.status_cb(USB_DC_SOF, NULL);
			UDPHS->UDPHS_CLRINT = UDPHS_CLRINT_INT_SOF;
			intsta &= ~UDPHS_INTSTA_INT_SOF;
		}

		if (intsta & UDPHS_INTSTA_MICRO_SOF) {
			dev_data.status_cb(USB_DC_SOF, NULL);
			UDPHS->UDPHS_CLRINT = UDPHS_CLRINT_MICRO_SOF;
			intsta &= ~UDPHS_INTSTA_MICRO_SOF;
		}
#endif // CONFIG_USB_DEVICE_SOF

		/* Suspend */
		/* This interrupt is always treated last (hence the '==') */
		if (intsta == UDPHS_INTSTA_DET_SUSPD) {
			LOG_DBG("SUSPEND");

			/* Enable wakeup */
			UDPHS->UDPHS_IEN |= UDPHS_IEN_WAKE_UP | UDPHS_IEN_ENDOFRSM;
			UDPHS->UDPHS_IEN &= ~UDPHS_IEN_DET_SUSPD;

			UDPHS->UDPHS_CLRINT = UDPHS_CLRINT_DET_SUSPD | UDPHS_CLRINT_WAKE_UP;

			dev_data.status_cb(USB_DC_SUSPEND, NULL);
		}

		/* Resume */
		else if ((intsta & UDPHS_INTSTA_WAKE_UP) || (intsta & UDPHS_INTSTA_ENDOFRSM)) {
			LOG_DBG("RESUME");

			dev_data.status_cb(USB_DC_RESUME, NULL);

			UDPHS->UDPHS_CLRINT = UDPHS_CLRINT_WAKE_UP | UDPHS_CLRINT_ENDOFRSM |
					      UDPHS_CLRINT_DET_SUSPD;

			UDPHS->UDPHS_IEN |= UDPHS_IEN_ENDOFRSM | UDPHS_IEN_DET_SUSPD;
			UDPHS->UDPHS_CLRINT = UDPHS_CLRINT_WAKE_UP | UDPHS_CLRINT_ENDOFRSM;
			UDPHS->UDPHS_IEN &= ~UDPHS_IEN_WAKE_UP;
		}

		/* End of bus reset */
		else if (intsta & UDPHS_INTSTA_ENDRESET) {
			LOG_DBG("RESET");

			usb_dc_reset();

			/* Flush and enable the Suspend interrupt */
			UDPHS->UDPHS_CLRINT = UDPHS_CLRINT_WAKE_UP | UDPHS_CLRINT_DET_SUSPD;

			dev_data.status_cb(USB_DC_RESET, NULL);

			UDPHS->UDPHS_CLRINT = UDPHS_CLRINT_ENDRESET;
			UDPHS->UDPHS_IEN |= UDPHS_IEN_DET_SUSPD;
		}

		/* Handle upstream resume interrupt */
		else if (intsta & UDPHS_INTSTA_UPSTR_RES) {
			LOG_DBG("WAKEUP");
			UDPHS->UDPHS_CLRINT = UDPHS_INTSTA_UPSTR_RES;
		}

		/* Endpoint interrupt */
		else {
			for (uint32_t ep_idx = 0; ep_idx < UDPHSEPT_NUMBER; ep_idx++) {
				if (intsta & (BIT(ep_idx) << 8)) {
					usb_dc_ep_isr(ep_idx);
				}
			}
		}

		/* Retrieve new interrupt status */
		intsta = UDPHS->UDPHS_INTSTA & UDPHS->UDPHS_IEN;
		LOG_DBG("intsta: %08x", intsta);
	}

	irq_enable(DT_INST_IRQN(0));
}

static int usb_dc_init(void)
{
	/* Configure the pull-up on D+ and disconnect it */
	UDPHS->UDPHS_CTRL |= UDPHS_CTRL_DETACH;
	UDPHS->UDPHS_CTRL |= UDPHS_CTRL_PULLD_DIS;

	/* Reset IP UDPHS */
	UDPHS->UDPHS_CTRL &= ~UDPHS_CTRL_EN_UDPHS;
	UDPHS->UDPHS_CTRL |= UDPHS_CTRL_EN_UDPHS;

	/* Disable DMA for UDPHS */
	for (uint32_t n = 1; n < UDPHSDMA_NUMBER; n++) {
		/* DMA stop channel command */
		UDPHS->UDPHS_DMA[n].UDPHS_DMACONTROL = 0; /* STOP command */

		/* Disable endpoint */
		UDPHS->UDPHS_EPT[n].UDPHS_EPTCTLDIS = BIT_MASK(32);

		/* Clear status endpoint */
		UDPHS->UDPHS_EPT[n].UDPHS_EPTCLRSTA = BIT_MASK(32);

		/* Reset endpoint config */
		UDPHS->UDPHS_EPT[n].UDPHS_EPTCTLENB = 0;

		/* Reset DMA channel (Buff count and Control field) */
		UDPHS->UDPHS_DMA[n].UDPHS_DMACONTROL = UDPHS_DMACONTROL_LDNXT_DSC;

		/* Reset DMA channel 0 (STOP) */
		UDPHS->UDPHS_DMA[n].UDPHS_DMACONTROL = 0;

		/* Clear DMA channel status (read the register for clear it) */
		UDPHS->UDPHS_DMA[n].UDPHS_DMASTATUS = UDPHS->UDPHS_DMA[n].UDPHS_DMASTATUS;
	}

	UDPHS->UDPHS_IEN = 0;
	UDPHS->UDPHS_CLRINT = UDPHS_CLRINT_UPSTR_RES | UDPHS_CLRINT_ENDOFRSM |
			      UDPHS_CLRINT_WAKE_UP | UDPHS_CLRINT_ENDRESET | UDPHS_CLRINT_INT_SOF |
			      UDPHS_CLRINT_MICRO_SOF | UDPHS_CLRINT_DET_SUSPD;

	/* Connect and enable the interrupt */
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), usb_dc_isr, 0, 0);
	irq_enable(DT_INST_IRQN(0));

	/* Enable interrupts */
	UDPHS->UDPHS_IEN = UDPHS_IEN_ENDOFRSM | UDPHS_IEN_WAKE_UP | UDPHS_IEN_DET_SUSPD;
}

int usb_dc_attach(void)
{
	LOG_DBG("attach");

	/* Start the peripheral clock */
	soc_pmc_peripheral_enable(DT_INST_PROP(0, peripheral_id));

	/* Enable the USB clock */
	usb_dc_enable_clock();

	usb_dc_init();

	/* Attach the device */
	UDPHS->UDPHS_CTRL &= ~UDPHS_CTRL_DETACH;
	UDPHS->UDPHS_CTRL |= UDPHS_CTRL_PULLD_DIS;

	return 0;
}

int usb_dc_detach(void)
{
	LOG_DBG("detach");

	UDPHS->UDPHS_CTRL |= UDPHS_CTRL_DETACH;
	UDPHS->UDPHS_CTRL &= ~UDPHS_CTRL_PULLD_DIS;

	/* Disable the USB clock */
	usb_dc_disable_clock();

	/* Disable the peripheral clock */
	soc_pmc_peripheral_disable(DT_INST_PROP(0, peripheral_id));

	/* Disable interrupt */
	irq_disable(DT_INST_IRQN(0));

	return 0;
}

int usb_dc_reset(void)
{
	LOG_DBG("reset");

	/* Reset & Disable USB Endpoints */
	for (uint32_t ep = 0; ep < UDPHSEPT_NUMBER; ep++) {
		UDPHS->UDPHS_EPT[ep].UDPHS_EPTCFG = 0;
		UDPHS->UDPHS_EPT[ep].UDPHS_EPTCTLDIS = UDPHS_EPTCTLDIS_EPT_DISABL;
	}

	UDPHS->UDPHS_EPTRST = BIT_MASK(UDPHSEPT_NUMBER);
	UDPHS->UDPHS_EPTRST = 0;

	/* Setup USB Interrupts */
	UDPHS->UDPHS_IEN = UDPHS_IEN_ENDRESET | UDPHS_IEN_DET_SUSPD |
#ifdef CONFIG_USB_DEVICE_SOF
			   UDPHS_IEN_INT_SOF | UDPHS_IEN_MICRO_SOF |
#endif
			   (BIT_MASK(UDPHSEPT_NUMBER) << 8);

	/* Setup Control Endpoint 0 */
	UDPHS->UDPHS_EPT[0].UDPHS_EPTCFG =
		UDPHS_EPTCFG_BK_NUMBER_1 | UDPHS_EPTCFG_EPT_TYPE_CTRL8 | UDPHS_EPTCFG_EPT_SIZE_64;
	UDPHS->UDPHS_EPT[0].UDPHS_EPTCTLENB =
		UDPHS_EPTCTLENB_RXRDY_TXKL | UDPHS_EPTCTLENB_TX_COMPLT | UDPHS_EPTCTLENB_RX_SETUP |
		UDPHS_EPTCTLENB_STALL_SNT | UDPHS_EPTCTLENB_NYET_DIS | UDPHS_EPTCTLENB_EPT_ENABL;

	return 0;
}

int usb_dc_set_address(const uint8_t addr)
{
	LOG_DBG("addr: %u", addr);

	UDPHS->UDPHS_CTRL &= ~UDPHS_CTRL_DEV_ADDR_Msk;
	UDPHS->UDPHS_CTRL |= (UDPHS_CTRL_FADDR_EN | UDPHS_CTRL_DEV_ADDR(addr));

	return 0;
}

void usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
	dev_data.status_cb = cb;
}

int usb_dc_ep_check_cap(const struct usb_dc_ep_cfg_data *const cfg)
{
	uint8_t ep_idx = USB_EP_GET_IDX(cfg->ep_addr);
	if (ep_idx >= UDPHSEPT_NUMBER) {
		LOG_ERR("Invalid EP index: %d", ep_idx);
		return -EINVAL;
	}

	switch (ep_idx) {
	case 0:
	case 3:
	case 4: {
		if (cfg->ep_mps > 64) {
			return -EINVAL;
		}
	} break;
	case 1:
	case 2: {
		if (cfg->ep_mps > 64) {
			return -EINVAL;
		}
	} break;
	case 5:
	case 6: {
		if (cfg->ep_mps > 1024) {
			return -EINVAL;
		}
	} break;
	default:
		return -EINVAL;
	}

	return 0;
}

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data *const cfg)
{
	uint8_t ep_idx = USB_EP_GET_IDX(cfg->ep_addr);
	if (ep_idx >= UDPHSEPT_NUMBER) {
		LOG_ERR("Invalid EP index: %d", ep_idx);
		return -EINVAL;
	}

	LOG_DBG("ep: 0x%02x, type: %d, mps: %d", cfg->ep_addr, cfg->ep_type, cfg->ep_mps);

	usb_dc_ep_fifo_reset(ep_idx);

	uint32_t eptcfg = 0U;

	if (cfg->ep_type == USB_DC_EP_ISOCHRONOUS) {
		eptcfg |= UDPHS_EPTCFG_BK_NUMBER_2;
	} else {
		eptcfg |= UDPHS_EPTCFG_BK_NUMBER_1;
	}

	switch (cfg->ep_type) {
	case USB_DC_EP_CONTROL:
		eptcfg |= UDPHS_EPTCFG_EPT_TYPE_CTRL8;
		break;
	case USB_DC_EP_ISOCHRONOUS:
		eptcfg |= UDPHS_EPTCFG_EPT_TYPE_ISO;
		break;
	case USB_DC_EP_BULK:
		eptcfg |= UDPHS_EPTCFG_EPT_TYPE_BULK;
		break;
	case USB_DC_EP_INTERRUPT:
		eptcfg |= UDPHS_EPTCFG_EPT_TYPE_INT;
		break;
	default:
		return -EINVAL;
	}

	if (USB_EP_DIR_IS_IN(cfg->ep_addr) && cfg->ep_type != USB_DC_EP_CONTROL) {
		eptcfg |= UDPHS_EPTCFG_EPT_DIR;
	}

	if (cfg->ep_mps <= 8) {
		eptcfg |= UDPHS_EPTCFG_EPT_SIZE_8;
	} else if (cfg->ep_mps <= 16) {
		eptcfg |= UDPHS_EPTCFG_EPT_SIZE_16;
	} else if (cfg->ep_mps <= 32) {
		eptcfg |= UDPHS_EPTCFG_EPT_SIZE_32;
	} else if (cfg->ep_mps <= 64) {
		eptcfg |= UDPHS_EPTCFG_EPT_SIZE_64;
	} else if (cfg->ep_mps <= 128) {
		eptcfg |= UDPHS_EPTCFG_EPT_SIZE_128;
	} else if (cfg->ep_mps <= 256) {
		eptcfg |= UDPHS_EPTCFG_EPT_SIZE_256;
	} else if (cfg->ep_mps <= 512) {
		eptcfg |= UDPHS_EPTCFG_EPT_SIZE_512;
	} else if (cfg->ep_mps <= 1024) {
		eptcfg |= UDPHS_EPTCFG_EPT_SIZE_1024;
	} else {
		eptcfg |= UDPHS_EPTCFG_EPT_SIZE_8;
	}
	dev_data.ep_data[ep_idx].mps = cfg->ep_mps;

	UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTCFG = eptcfg;

	UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTCTLENB =
		UDPHS_EPTCTLENB_RXRDY_TXKL | UDPHS_EPTCTLENB_TX_COMPLT | UDPHS_EPTCTLENB_STALL_SNT;

	return 0;
}

int usb_dc_ep_set_stall(const uint8_t ep)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	if (ep_idx >= UDPHSEPT_NUMBER) {
		LOG_ERR("Invalid EP index: %d", ep_idx);
		return -EINVAL;
	}

	LOG_DBG("ep: 0x%02x", ep);

	if (dev_data.ep_data[ep_idx].stalled) {
		return 0;
	}

	UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTSETSTA = UDPHS_EPTSETSTA_FRCESTALL;
	UDPHS->UDPHS_IEN |= BIT(ep_idx) << 8;

	dev_data.ep_data[ep_idx].stalled = 1;

	return 0;
}

int usb_dc_ep_clear_stall(const uint8_t ep)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	if (ep_idx >= UDPHSEPT_NUMBER) {
		LOG_ERR("Invalid EP index: %d", ep_idx);
		return -EINVAL;
	}

	LOG_DBG("ep: 0x%02x", ep);

	if (!dev_data.ep_data[ep_idx].stalled) {
		return 0;
	}

	UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTCLRSTA =
		UDPHS_EPTCLRSTA_TOGGLESQ | UDPHS_EPTCLRSTA_FRCESTALL;
	UDPHS->UDPHS_EPTRST = BIT(ep_idx);

	dev_data.ep_data[ep_idx].stalled = 0;

	return 0;
}

int usb_dc_ep_is_stalled(const uint8_t ep, uint8_t *const stalled)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	if (ep_idx >= UDPHSEPT_NUMBER) {
		LOG_ERR("Invalid EP index: %d", ep_idx);
		return -EINVAL;
	}

	*stalled = dev_data.ep_data[ep_idx].stalled;

	return 0;
}

int usb_dc_ep_halt(const uint8_t ep)
{
	LOG_DBG("ep: 0x%02x", ep);
	return usb_dc_ep_set_stall(ep);
}

int usb_dc_ep_enable(const uint8_t ep)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	if (ep_idx >= UDPHSEPT_NUMBER) {
		LOG_ERR("Invalid EP index: %d", ep_idx);
		return -EINVAL;
	}

	LOG_DBG("ep: 0x%02x", ep);

	UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTCTLENB = UDPHS_EPTCTLENB_EPT_ENABL;
	UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTCLRSTA =
		UDPHS_EPTCLRSTA_TOGGLESQ | UDPHS_EPTCLRSTA_FRCESTALL;

	UDPHS->UDPHS_EPTRST |= BIT(ep_idx);
	UDPHS->UDPHS_EPTRST &= ~BIT(ep_idx);

	return 0;
}

int usb_dc_ep_disable(const uint8_t ep)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	if (ep_idx >= UDPHSEPT_NUMBER) {
		LOG_ERR("Invalid EP index: %d", ep_idx);
		return -EINVAL;
	}

	LOG_DBG("ep: 0x%02x", ep);

	UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTCTLDIS = UDPHS_EPTCTLDIS_EPT_DISABL;

	return 0;
}

int usb_dc_ep_flush(const uint8_t ep)
{
	LOG_DBG("ep: 0x%02x", ep);
	return 0;
}

int usb_dc_ep_write(const uint8_t ep, const uint8_t *const data, const uint32_t data_len,
		    uint32_t *const ret_bytes)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	if (ep_idx >= UDPHSEPT_NUMBER) {
		LOG_ERR("Invalid EP index: %d", ep_idx);
		return -EINVAL;
	}

	if (USB_EP_GET_DIR(ep) != USB_EP_DIR_IN) {
		LOG_ERR("Invalid EP direction: %d", ep_idx);
		return -EINVAL;
	}

	uint32_t packet_len = MIN(data_len, dev_data.ep_data[ep_idx].mps);
	for (int i = 0; i < packet_len; i++) {
		usb_dc_ep_fifo_put(ep_idx, data[i]);
	}
	__DSB();

	UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTSETSTA = UDPHS_EPTSETSTA_TXRDY;

	if (ret_bytes) {
		*ret_bytes = packet_len;
	}

	LOG_DBG("ep: 0x%02x, len: %d of %d", ep, packet_len, data_len);
	LOG_HEXDUMP_DBG(data, packet_len, "data:");

	return 0;
}

int usb_dc_ep_read(const uint8_t ep, uint8_t *const data, const uint32_t max_data_len,
		   uint32_t *const read_bytes)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	if (ep_idx >= UDPHSEPT_NUMBER) {
		LOG_ERR("Invalid EP index: %d", ep_idx);
		return -EINVAL;
	}

	if (USB_EP_GET_DIR(ep) != USB_EP_DIR_OUT) {
		LOG_ERR("Invalid EP direction: %d", ep_idx);
		return -EINVAL;
	}

	uint32_t data_len = (UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTSTA & UDPHS_EPTSTA_BYTE_COUNT_Msk) >>
			    UDPHS_EPTSTA_BYTE_COUNT_Pos;

	LOG_DBG("ep: 0x%02x, len: %d", ep, data_len);

	if (!data && !max_data_len) {
		/*
		 * When both buffer and max data to read are zero return
		 * the available data in buffer.
		 */
		if (read_bytes) {
			*read_bytes = data_len;
		}
		return 0;
	}

	if (data_len > max_data_len) {
		LOG_WRN("Not enough space to copy all the data: %d", ep_idx);
		data_len = max_data_len;
	}

	if (data != NULL) {
		for (int i = 0; i < data_len; i++) {
			data[i] = usb_dc_ep_fifo_get(ep_idx);
		}
	}

	LOG_HEXDUMP_DBG(data, data_len, "data:");

	if (UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTSTA & UDPHS_EPTSTA_RXRDY_TXKL) {
		LOG_DBG("clear RXRDY_TXKL");
		UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTCLRSTA = UDPHS_EPTCLRSTA_RXRDY_TXKL;
	}

	if (UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTSTA & UDPHS_EPTSTA_RX_SETUP) {
		LOG_DBG("clear RX_SETUP");
		UDPHS->UDPHS_EPT[ep_idx].UDPHS_EPTCLRSTA = UDPHS_EPTCLRSTA_RX_SETUP;
	}

	UDPHS->UDPHS_IEN |= (BIT(ep_idx) << 8);

	if (read_bytes) {
		*read_bytes = data_len;
	}

	return 0;
}

int usb_dc_ep_set_callback(const uint8_t ep, const usb_dc_ep_callback cb)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	if (ep_idx >= UDPHSEPT_NUMBER) {
		LOG_ERR("Invalid EP index: %d", ep_idx);
		return -EINVAL;
	}

	if (USB_EP_DIR_IS_IN(ep)) {
		dev_data.ep_data[ep_idx].cb_in = cb;
	} else {
		dev_data.ep_data[ep_idx].cb_out = cb;
	}

	return 0;
}

int usb_dc_ep_read_wait(uint8_t ep, uint8_t *data, uint32_t max_data_len, uint32_t *read_bytes)
{
	LOG_DBG("ep: 0x%02x", ep);
	return -ENOTSUP;
}

int usb_dc_ep_read_continue(uint8_t ep)
{
	LOG_DBG("ep: 0x%02x", ep);
	return -ENOTSUP;
}

int usb_dc_ep_mps(uint8_t ep)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	if (ep_idx >= UDPHSEPT_NUMBER) {
		LOG_ERR("Invalid EP index: %d", ep_idx);
		return -EINVAL;
	}

	return dev_data.ep_data[ep_idx].mps;
}

int usb_dc_wakeup_request(void)
{
	UDPHS->UDPHS_IEN |= UDPHS_IEN_UPSTR_RES;
	UDPHS->UDPHS_CTRL |= UDPHS_CTRL_REWAKEUP;
	return 0;
}
