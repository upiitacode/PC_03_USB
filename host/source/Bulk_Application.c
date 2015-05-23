#include <stdio.h>
#include <string.h>
#include <libusb.h>

#define OUT_BUFFER_SIZE 64
#define IN_BUFFER_SIZE 64
int main() {
	char buff_in[IN_BUFFER_SIZE];
	char buff_out[OUT_BUFFER_SIZE];
	int dwlen_out;
	int dwlen_in = IN_BUFFER_SIZE;
	int pdwlen_out;
	int pdwlen_in;
	int readVal;
	libusb_context *ctx = NULL; //context =default
	libusb_device_handle* myHandle = NULL;
	int opFailed;
	opFailed = libusb_init(&ctx);
	if (opFailed < 0) {
		printf("initialization failed\n");
	} else {
		printf("initialization success\n");
		myHandle = libusb_open_device_with_vid_pid(ctx, 0x1CBE, 0x0003);
		if (myHandle != NULL) {
			printf("device is open. handle = %p\n", myHandle);
			libusb_claim_interface(myHandle, 0);
			//write and read operations
			opFailed = libusb_bulk_transfer(myHandle, 0x81,(unsigned char *) buff_in, dwlen_in, &pdwlen_in, 10);
			if (opFailed == 0) {
				printf("read success.\n");
				printf("%d of %d bytes read\n", pdwlen_in, dwlen_in);
				readVal=(((int *)buff_in)[0]);
				printf("data: 0x%03X\n", readVal);
			} else {
				printf("read failed.\n");
			}
		} else {
			printf("device not open.\n");
		}
		libusb_close(myHandle);
	}
	return 0;
}
