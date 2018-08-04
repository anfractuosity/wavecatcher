#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/common/spi_common_v2.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>

/*

    Wavecatcher firmware 
    
    Based on a number of libopencm3 examples (SPI DMA example and CDC example)

*/


//#ifndef USE_16BIT_TRANSFERS
///#define USE_16BIT_TRANSFERS 1
//#endif

#define READ 64 

int counter_tx = 0;
int counter_rx = 0;
unsigned char tot = 0;

uint8_t tx_packet[64] = { 0 };
uint8_t rx_packet[64] = { 0 };
uint8_t dx_packet[64] = { 0 };

int transc = 2;
int read = 0;

/* This is for the counter state flag */
typedef enum {
    TX_UP_RX_HOLD = 0,
    TX_HOLD_RX_UP,
    TX_DOWN_RX_DOWN
} cnt_state;

/* This is a global spi state flag */
typedef enum {
    NONE = 0,
    ONE,
    DONE
} trans_status;

volatile trans_status transceive_status;

static const struct usb_device_descriptor dev = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = USB_CLASS_CDC,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x0483,
    .idProduct = 0x5740,
    .bcdDevice = 0x0200,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux
 * cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = { {
                                                             .bLength =
                                                             USB_DT_ENDPOINT_SIZE,
                                                             .bDescriptorType =
                                                             USB_DT_ENDPOINT,
                                                             .bEndpointAddress =
                                                             0x83,
                                                             .bmAttributes =
                                                             USB_ENDPOINT_ATTR_INTERRUPT,
                                                             .wMaxPacketSize =
                                                             16,
                                                             .bInterval = 255,
                                                             }
};

static const struct usb_endpoint_descriptor data_endp[] = { {
                                                             .bLength =
                                                             USB_DT_ENDPOINT_SIZE,
                                                             .bDescriptorType =
                                                             USB_DT_ENDPOINT,
                                                             .bEndpointAddress =
                                                             0x01,
                                                             .bmAttributes =
                                                             USB_ENDPOINT_ATTR_BULK,
                                                             .wMaxPacketSize =
                                                             64,
                                                             .bInterval = 1,
                                                             }, {
                                                                 .bLength =
                                                                 USB_DT_ENDPOINT_SIZE,
                                                                 .bDescriptorType
                                                                 =
                                                                 USB_DT_ENDPOINT,
                                                                 .bEndpointAddress
                                                                 = 0x82,
                                                                 .bmAttributes =
                                                                 USB_ENDPOINT_ATTR_BULK,
                                                                 .wMaxPacketSize
                                                                 = 64,
                                                                 .bInterval = 1,
                                                                 }
};

static const struct {
    struct usb_cdc_header_descriptor header;
    struct usb_cdc_call_management_descriptor call_mgmt;
    struct usb_cdc_acm_descriptor acm;
    struct usb_cdc_union_descriptor cdc_union;
} __attribute__ ((packed)) cdcacm_functional_descriptors = {
    .header = {
    .bFunctionLength =
            sizeof(struct usb_cdc_header_descriptor),.bDescriptorType =
            CS_INTERFACE,.bDescriptorSubtype = USB_CDC_TYPE_HEADER,.bcdCDC =
            0x0110,},.call_mgmt = {
    .bFunctionLength =
            sizeof(struct
                       usb_cdc_call_management_descriptor),.bDescriptorType
            = CS_INTERFACE,.bDescriptorSubtype =
            USB_CDC_TYPE_CALL_MANAGEMENT,.bmCapabilities =
            0,.bDataInterface = 1,},.acm = {
    .bFunctionLength =
            sizeof(struct usb_cdc_acm_descriptor),.bDescriptorType =
            CS_INTERFACE,.bDescriptorSubtype =
            USB_CDC_TYPE_ACM,.bmCapabilities = 0,},.cdc_union = {
.bFunctionLength =
            sizeof(struct usb_cdc_union_descriptor),.bDescriptorType =
            CS_INTERFACE,.bDescriptorSubtype =
            USB_CDC_TYPE_UNION,.bControlInterface =
            0,.bSubordinateInterface0 = 1,},};

static const struct usb_interface_descriptor comm_iface[] = { {
                                                               .bLength =
                                                               USB_DT_INTERFACE_SIZE,
                                                               .bDescriptorType
                                                               =
                                                               USB_DT_INTERFACE,
                                                               .bInterfaceNumber
                                                               = 0,
                                                               .bAlternateSetting
                                                               = 0,
                                                               .bNumEndpoints =
                                                               1,
                                                               .bInterfaceClass
                                                               = USB_CLASS_CDC,
                                                               .bInterfaceSubClass
                                                               =
                                                               USB_CDC_SUBCLASS_ACM,
                                                               .bInterfaceProtocol
                                                               =
                                                               USB_CDC_PROTOCOL_AT,
                                                               .iInterface = 0,

                                                               .endpoint =
                                                               comm_endp,

                                                               .extra =
                                                               &cdcacm_functional_descriptors,
                                                               .extralen =
                                                               sizeof
                                                               (cdcacm_functional_descriptors),
                                                               }
};

static const struct usb_interface_descriptor data_iface[] = { {
                                                               .bLength =
                                                               USB_DT_INTERFACE_SIZE,
                                                               .bDescriptorType
                                                               =
                                                               USB_DT_INTERFACE,
                                                               .bInterfaceNumber
                                                               = 1,
                                                               .bAlternateSetting
                                                               = 0,
                                                               .bNumEndpoints =
                                                               2,
                                                               .bInterfaceClass
                                                               = USB_CLASS_DATA,
                                                               .bInterfaceSubClass
                                                               = 0,
                                                               .bInterfaceProtocol
                                                               = 0,
                                                               .iInterface = 0,

                                                               .endpoint =
                                                               data_endp,
                                                               }
};

static const struct usb_interface ifaces[] = { {
                                                .num_altsetting = 1,
                                                .altsetting = comm_iface,
                                                }, {
                                                    .num_altsetting = 1,
                                                    .altsetting = data_iface,
                                                    }
};

static const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 2,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 0x32,

    .interface = ifaces,
};

static const char *usb_strings[] = {
    "Anfractuosity",
    "Wavecatcher",
    "Wavecatcher",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes cdcacm_control_request(usbd_device *
                                                             usbd_dev, struct
                                                             usb_setup_data
                                                             *req,
                                                             uint8_t ** buf,
                                                             uint16_t * len,
                                                             void (**complete)
                                                              (usbd_device *
                                                               usbd_dev,
                                                               struct
                                                               usb_setup_data *
                                                               req)) {
    (void)complete;
    (void)buf;
    (void)usbd_dev;

    switch (req->bRequest) {
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE:{
            /*
             * This Linux cdc_acm driver requires this to be implemented
             * even though it's optional in the CDC spec, and we don't
             * advertise it in the ACM functional descriptor.
             */
            char local_buf[10];
            struct usb_cdc_notification *notif = (void *)local_buf;

            /* We echo signals back to host as notification. */
            notif->bmRequestType = 0xA1;
            notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
            notif->wValue = 0;
            notif->wIndex = 0;
            notif->wLength = 2;
            local_buf[8] = req->wValue & 3;
            local_buf[9] = 0;
            // usbd_ep_write_packet(0x83, buf, 10);
            return USBD_REQ_HANDLED;
        }
    case USB_CDC_REQ_SET_LINE_CODING:
        if (*len < sizeof(struct usb_cdc_line_coding))
            return USBD_REQ_NOTSUPP;
        return USBD_REQ_HANDLED;
    }
    return USBD_REQ_NOTSUPP;
}

static void cdcacm_data_rx_cb(usbd_device * usbd_dev, uint8_t ep) {
    (void)ep;
    (void)usbd_dev;

    char buf[64];
    int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

    if (len) {
        usbd_ep_write_packet(usbd_dev, 0x82, buf, len);
        buf[len] = 0;
    }

}

int z = 0;

static void cdcacm_data_tx_cb(usbd_device * usbd_dev, uint8_t ep) {
    (void)ep;
    (void)usbd_dev;
    uint8_t buf[64 + 1] __attribute__ ((aligned(2)));

    if (read == 1) {
        memcpy(buf, dx_packet, READ);
        read = 0;
    }
    usbd_ep_write_packet(usbd_dev, 0x82, (unsigned char *)buf, READ);
}

static void cdcacm_set_config(usbd_device * usbd_dev, uint16_t wValue) {
    (void)wValue;
    (void)usbd_dev;

    usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, NULL);
    usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64,
                  cdcacm_data_tx_cb);
    usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

    usbd_register_control_callback(usbd_dev,
                                   USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
                                   USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
                                   cdcacm_control_request);

    cdcacm_data_tx_cb(usbd_dev, 0x82);

}

static void usb_setup(void) {
    rcc_periph_clock_enable(RCC_USB);
    rcc_periph_clock_enable(RCC_GPIOA);
    /* Setup GPIO pins for USB D+/D-. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
    gpio_set_af(GPIOA, GPIO_AF14, GPIO11 | GPIO12);
}

static void spi_setup(void) {
    rcc_periph_clock_enable(RCC_SPI1);
    /* For spi signal pins */
    rcc_periph_clock_enable(RCC_GPIOA);
    /* For spi mode select on the l3gd20 */
    rcc_periph_clock_enable(RCC_GPIOE);
    /* Setup GPIO pins for AF5 for SPI1 signals. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6 | GPIO7);
    gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);

    //spi initialization;
    spi_set_master_mode(SPI1);
    spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_16);  //72 / 16 = 4.5MHz
    spi_set_clock_polarity_1(SPI1);
    spi_set_clock_phase_1(SPI1);
    spi_set_full_duplex_mode(SPI1);
    spi_set_unidirectional_mode(SPI1);  
    spi_set_data_size(SPI1, SPI_CR2_DS_8BIT);
    spi_enable_software_slave_management(SPI1);
    spi_send_msb_first(SPI1);
    spi_set_nss_high(SPI1);
    spi_enable_ss_output(SPI1);
    spi_fifo_reception_threshold_8bit(SPI1);
    SPI_I2SCFGR(SPI1) &= ~SPI_I2SCFGR_I2SMOD;

    spi_enable(SPI1);
}

const struct rcc_clock_scale this_clock_config = {
    .pllsrc = RCC_CFGR_PLLSRC_HSE_PREDIV,
    .pllmul = RCC_CFGR_PLLMUL_MUL9,
    .plldiv = RCC_CFGR2_PREDIV_NODIV,
    .usbdiv1 = false,
    .flash_waitstates = 2,
    .hpre = RCC_CFGR_HPRE_DIV_NONE,
    .ppre1 = RCC_CFGR_PPRE1_DIV_2,
    .ppre2 = RCC_CFGR_PPRE2_DIV_NONE,
    .ahb_frequency = 72e6,
    .apb1_frequency = 32e6,
    .apb2_frequency = 72e6,
};

static void dma_int_enable(void) {
    /* SPI1 RX on DMA1 Channel 2 */
    nvic_set_priority(NVIC_DMA1_CHANNEL2_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_CHANNEL2_IRQ);
    /* SPI1 TX on DMA1 Channel 3 */
    nvic_set_priority(NVIC_DMA1_CHANNEL3_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_CHANNEL3_IRQ);
}

static void dma_setup(void) {
    dma_int_enable();
}

#if USE_16BIT_TRANSFERS
static int spi_dma_transceive(uint16_t * tx_buf, int tx_len, uint16_t * rx_buf,
                              int rx_len)
#else
static int spi_dma_transceive(uint8_t * tx_buf, int tx_len, uint8_t * rx_buf,
                              int rx_len)
#endif
{
    /* Check for 0 length in both tx and rx */
    if ((rx_len < 1) && (tx_len < 1)) {
        /* return -1 as error */
        return -1;
    }

    /* Reset DMA channels */
    dma_channel_reset(DMA1, DMA_CHANNEL2);
    dma_channel_reset(DMA1, DMA_CHANNEL3);

    /* Reset SPI data and status registers.
     * Here we assume that the SPI peripheral is NOT
     * busy any longer, i.e. the last activity was verified
     * complete elsewhere in the program.
     */
    volatile uint8_t temp_data __attribute__ ((unused));
    while (SPI_SR(SPI1) & (SPI_SR_RXNE | SPI_SR_OVR)) {
        temp_data = SPI_DR(SPI1);
    }

    /* Reset status flag appropriately (both 0 case caught above) */
    transceive_status = NONE;
    if (rx_len < 1) {
        transceive_status = ONE;
    }
    if (tx_len < 1) {
        transceive_status = ONE;
    }

    /* Set up rx dma, note it has higher priority to avoid overrun */
    if (rx_len > 0) {

        dma_set_peripheral_address(DMA1, DMA_CHANNEL2, (uint32_t) & SPI1_DR);
        dma_set_memory_address(DMA1, DMA_CHANNEL2, (uint32_t) rx_buf);
        dma_set_number_of_data(DMA1, DMA_CHANNEL2, rx_len);
        dma_set_read_from_peripheral(DMA1, DMA_CHANNEL2);
        dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL2);
#if USE_16BIT_TRANSFERS
        dma_set_peripheral_size(DMA1, DMA_CHANNEL2, DMA_CCR_PSIZE_16BIT);
        dma_set_memory_size(DMA1, DMA_CHANNEL2, DMA_CCR_MSIZE_16BIT);
#else
        dma_set_peripheral_size(DMA1, DMA_CHANNEL2, DMA_CCR_PSIZE_8BIT);
        dma_set_memory_size(DMA1, DMA_CHANNEL2, DMA_CCR_MSIZE_8BIT);
#endif
        dma_set_priority(DMA1, DMA_CHANNEL2, DMA_CCR_PL_VERY_HIGH);
    }

    /* Set up tx dma */
    if (tx_len > 0) {
        dma_set_peripheral_address(DMA1, DMA_CHANNEL3, (uint32_t) & SPI1_DR);
        dma_set_memory_address(DMA1, DMA_CHANNEL3, (uint32_t) tx_buf);
        dma_set_number_of_data(DMA1, DMA_CHANNEL3, tx_len);
        dma_set_read_from_memory(DMA1, DMA_CHANNEL3);
        dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL3);
#if USE_16BIT_TRANSFERS
        dma_set_peripheral_size(DMA1, DMA_CHANNEL3, DMA_CCR_PSIZE_16BIT);
        dma_set_memory_size(DMA1, DMA_CHANNEL3, DMA_CCR_MSIZE_16BIT);
#else
        dma_set_peripheral_size(DMA1, DMA_CHANNEL3, DMA_CCR_PSIZE_8BIT);
        dma_set_memory_size(DMA1, DMA_CHANNEL3, DMA_CCR_MSIZE_8BIT);
#endif
        dma_set_priority(DMA1, DMA_CHANNEL3, DMA_CCR_PL_HIGH);
    }

    /* Enable dma transfer complete interrupts */
    if (rx_len > 0) {

        dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL2);
    }
    if (tx_len > 0) {
        dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL3);
    }

    /* Activate dma channels */
    if (rx_len > 0) {
        dma_enable_channel(DMA1, DMA_CHANNEL2);
    }
    if (tx_len > 0) {
        dma_enable_channel(DMA1, DMA_CHANNEL3);
    }

    /* Enable the spi transfer via dma
     * This will immediately start the transmission,
     * after which when the receive is complete, the
     * receive dma will activate
     */
    if (rx_len > 0) {
        spi_enable_rx_dma(SPI1);
    }
    if (tx_len > 0) {
        spi_enable_tx_dma(SPI1);
    }

    return 1;
}

/* SPI receive completed with DMA */
void dma1_channel2_isr(void) {
    if ((DMA1_ISR & DMA_ISR_TCIF2) != 0) {
        DMA1_IFCR |= DMA_IFCR_CTCIF2;
    }

    dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL2);
    spi_disable_rx_dma(SPI1);
    dma_disable_channel(DMA1, DMA_CHANNEL2);

    memcpy(dx_packet, rx_packet, READ);
    read = 1;

    transc++;

}

/* SPI transmit completed with DMA */
void dma1_channel3_isr(void) {
    if ((DMA1_ISR & DMA_ISR_TCIF3) != 0) {
        DMA1_IFCR |= DMA_IFCR_CTCIF3;
    }

    dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL3);
    spi_disable_tx_dma(SPI1);
    dma_disable_channel(DMA1, DMA_CHANNEL3);
    transc++;
}

int main(void) {
    int i;

    usbd_device *usbd_dev;
    spi_setup();
    dma_setup();
    rcc_clock_setup_pll(&this_clock_config);
    rcc_periph_clock_enable(RCC_GPIOA);

    /* Enable SPI1 Periph and gpio clocks */
    rcc_periph_clock_enable(RCC_SPI1);

    /* Enable DMA1 clock */
    rcc_periph_clock_enable(RCC_DMA1);

    /*
     * Vile hack to reenumerate, physically _drag_ d+ low.
     * do NOT do this if you're board has proper usb pull up control!
     * (need at least 2.5us to trigger usb disconnect)
     */
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
    gpio_clear(GPIOA, GPIO12);
    for (unsigned int i = 0; i < 800000; i++) {
        __asm__("nop");
    }

    usb_setup();
    usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings,
                         3, usbd_control_buffer, sizeof(usbd_control_buffer));
    usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);

    for (i = 0; i < 0x800000; i++)
        __asm__("nop");

    while (1) {

        if (transc == 2 && spi_dma_transceive(tx_packet, READ, rx_packet, READ)) {
            transc = 0;
        }

        usbd_poll(usbd_dev);

    }
}
