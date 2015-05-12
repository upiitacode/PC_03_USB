#include <stdio.h>
#include <string.h>
#include <libusb.h>
int main(){
    char buffString[20];
    char buff_in[128];
    char buff_out[128];
    int dwlen_out;
    int dwlen_in=64;
    int pdwlen_out;
    int pdwlen_in;
    libusb_context *ctx = NULL;//conetex =default
    libusb_device_handle* myHandle=NULL;
    int r;
    int opFailed;
    r = libusb_init(&ctx);
    if(r<0){
          printf("initialization failed\n");    
    }else{
          printf("initializacion succes\n");
          //libusb_set_debug(ctx,4);
          myHandle=libusb_open_device_with_vid_pid(ctx,0x1CBE,0x0003);
          if(myHandle!=NULL){
              printf("device is open. handle = %p\n",myHandle);
              libusb_claim_interface(myHandle,0);
              strcpy(buff_out,"hola MUndo");
              printf("Enter String: ");
              gets(buff_out);
              dwlen_out=strlen(buff_out)+1;
              //write and read operations
              opFailed=libusb_bulk_transfer(myHandle,0x01,buff_out,dwlen_out,&pdwlen_out,1000);
              if(opFailed==0){
                printf("write succes.\n");
                printf("%d of %d bytes written\n",pdwlen_out,dwlen_out);
              }else{
                printf("write failed.\n");  
              }   
              opFailed=libusb_bulk_transfer(myHandle,0x81,buff_in,dwlen_in,&pdwlen_in,10);
              if(opFailed==0){
                  printf("read succes.\n");
                  printf("%d of %d bytes read\n",pdwlen_in,dwlen_in);
                  printf("data: %s\n",buff_in);
              }else{
                  printf("read failed.\n");
              }          
          }else{
              printf("device not open.\n");               
          }
          libusb_close(myHandle);
    }
    printf("Press any key to exit");
    gets(buffString);
    return 0;
}
