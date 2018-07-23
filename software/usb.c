// Read device via libusb, which works better than accessing data from /dev/ttyACM0
// Based on:
// http://www.bertos.org/use/tutorial-front-page/drivers-usb-device

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <libusb-1.0/libusb.h>
#define USB_VENDOR_ID	    0x0483      
#define USB_PRODUCT_ID	    0x5740    
#define USB_ENDPOINT_IN	   0x82 
#define USB_TIMEOUT	        3000        /* Connection timeout (in ms) */

static libusb_context *ctx = NULL;
static libusb_device_handle *handle;

static uint8_t receiveBuf[64];
uint8_t transferBuf[64];


pthread_mutex_t lock;

int countSetBits(unsigned int n)
{
  n = n - ((n >> 1) & 0x55555555);
  n = (n & 0x33333333) + ((n >> 2) & 0x33333333);
  n = (n + (n >> 4)) & 0x0F0F0F0F;
  n = n + (n >> 8);
  n = n + (n >> 16);
  return n & 0x0000003F;
}
 
FILE *x ;
/*
 * Read a packet
 */
static int usb_read(void)
{
	int nread, ret;
	ret = libusb_bulk_transfer(handle, USB_ENDPOINT_IN, receiveBuf, sizeof(receiveBuf),
			&nread, USB_TIMEOUT);
	if (ret){
		printf("ERROR in bulk read: %d\n", ret);
		return -1;
    } else{
        fwrite(receiveBuf,nread,1,x);
		return nread;
    }
}


uint64_t bytes = 0;
uint64_t old = 0;

/*
 * on SIGINT: close USB interface
 * This still leads to a segfault on my system...
 */
static void sighandler(int signum)
{
    printf( "\nInterrupt signal received\n" );
	if (handle){
        libusb_release_interface (handle, 0);
        printf( "\nInterrupt signal received1\n" );
        libusb_close(handle);
        printf( "\nInterrupt signal received2\n" );
	}
	printf( "\nInterrupt signal received3\n" );
	libusb_exit(NULL);
	printf( "\nInterrupt signal received4\n" );

	exit(0);
}

int main(int argc, char **argv)
{

    x = fopen("dump.bin","ab");
    //Pass Interrupt Signal to our handler
	signal(SIGINT, sighandler);
	libusb_init(&ctx);
	libusb_set_debug(ctx, 3);

    //Open Device with VendorID and ProductID
	handle = libusb_open_device_with_vid_pid(ctx,
				USB_VENDOR_ID, USB_PRODUCT_ID);
	if (!handle) {
		perror("device not found");
		return 1;
	}

	int r = 1;

	//Claim Interface 0 from the device
	libusb_detach_kernel_driver(handle,0);

	r = libusb_claim_interface(handle, 0);
	if (r < 0) {
		fprintf(stderr, "usb_claim_interface error %d\n", r);
		return 2;
	}

	printf("Interface claimed\n");

    unsigned long bytes = 0;
	while (1){
		bytes += usb_read();
        if (bytes >= 1024 * 1024 * 100)
            break;
	}

	libusb_close(handle);
	libusb_exit(NULL);

	return 0;
}
