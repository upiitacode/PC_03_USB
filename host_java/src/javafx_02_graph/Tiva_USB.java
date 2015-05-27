package javafx_02_graph;


import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.IntBuffer;
import org.usb4java.BufferUtils;
import org.usb4java.Context;
import org.usb4java.DeviceHandle;
import org.usb4java.LibUsb;
import org.usb4java.LibUsbException;

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author ubuntu
 */
public class Tiva_USB {
    
    /** The vendor ID */
    private static final short VENDOR_ID = 0x1cbe;
    /** The product ID */
    private static final short PRODUCT_ID = 0x0003;
     /** The interface  */
    private static final byte INTERFACE = 0;
    /** The ADB input endpoint  */
    private static final byte IN_ENDPOINT = (byte) 0x81;
    /** The ADB output endpoint */
    private static final byte OUT_ENDPOINT = 0x01;
    /** The communication timeout in milliseconds. */
    private static final int TIMEOUT = 1000;
    
    private Context context;
    private DeviceHandle handle;
    
    public boolean connect(){
        context = new Context();

        // Initialize the libusb context
        int result = LibUsb.init(context);
        if (result < 0)
        {
            throw new LibUsbException("Unable to initialize libusb", result);
        }
        
        // Open test device (Samsung Galaxy Nexus)
        handle = LibUsb.openDeviceWithVidPid(null, VENDOR_ID,
                PRODUCT_ID);
        if (null == handle)
        {
            System.err.println("Test device not found.");
            System.exit(1);
        }
        
        // Claim the ADB interface
        result = LibUsb.claimInterface(handle, INTERFACE);
        if (result != LibUsb.SUCCESS)
        {
            throw new LibUsbException("Unable to claim interface", result);
        }
           
        return true;
    }
    
    public boolean disconnect(){
        // Release the ADB interface
        int result = LibUsb.releaseInterface(handle, INTERFACE);
        if (result != LibUsb.SUCCESS)
        {
            throw new LibUsbException("Unable to release interface", result);
        }

        // Close the device
        LibUsb.close(handle);

        // Deinitialize the libusb context
        LibUsb.exit(context);
        return true;
    }
    
    public int read(){
        // Read Data from usb 
        ByteBuffer buffer= readUSB(handle, 4);
        return buffer.getInt(0);
    }
    
    public void write(char c){
        
    }
    
    public static ByteBuffer readUSB(DeviceHandle handle, int size){
        ByteBuffer buffer = BufferUtils.allocateByteBuffer(size).order(
            ByteOrder.LITTLE_ENDIAN);
        IntBuffer transferred = BufferUtils.allocateIntBuffer();
        int result = LibUsb.bulkTransfer(handle, IN_ENDPOINT, buffer,
            transferred, TIMEOUT);
        if (result != LibUsb.SUCCESS)
        {
            throw new LibUsbException("Unable to read data", result);
        }
        return buffer;
    }
    
}
