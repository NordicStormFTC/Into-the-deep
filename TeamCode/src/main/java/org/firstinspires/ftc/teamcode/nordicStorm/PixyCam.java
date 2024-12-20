package org.firstinspires.ftc.teamcode.nordicStorm;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayList;


/**
 * @ annotations allow us to register Pixy2 as a device when configuring robot on driver hub.
 * note that MCP9808 is in fact the XML tag for the adafruit temperature I2C Sensor. I was not able to find
 * the appropriate tag for Pixy2 and used as shown in this demonstration 'writing an I2C Device Driver'
 * https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Writing-an-I2C-Driver
 **/

@I2cDeviceType
@DeviceProperties(name = "Pixy2 Smart Camera", description = "Pixy2 Smart Camera", xmlTag = "MCP9808")
public class PixyCam extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynch, PixyCam.PixyCamParams> {


    /// the default address for Pixy2s I2C interface is 0x54
    static final I2cAddr DEFAULT_ADDRESS = I2cAddr.create7bit(0x54);


    public ArrayList<PixyBlock> detectedBlocks = new ArrayList<>();


    public static final int BLOCK_SIZE = 14;

    /// this is the inner class used to comply with FTC SDK Architecture
    public static class PixyCamParams implements Cloneable {

        I2cAddr i2cAddr = DEFAULT_ADDRESS;

        public PixyCamParams clone() {
            try {
                return (PixyCamParams) super.clone();
            } catch (CloneNotSupportedException e) {
                throw new RuntimeException("Internal error: parameters not cloneable");
            }
        }
    }

    public PixyCam(I2cDeviceSynch deviceSynch) {
        super(deviceSynch, true, new PixyCamParams());

        this.deviceClient.setI2cAddress(DEFAULT_ADDRESS);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    

    /**
     * @return an updated list of all "pixy block" objects registered by the camera
     */
    public ArrayList<PixyBlock> read() {

        /// this clears the results of stale blocks
        detectedBlocks.clear();
        //detectedBlocks = new ArrayList<>();

        /// pixy2 has 26 bytes for us to read through
        byte[] bytes = new byte[26];

        /// this sets the I2C device, in this case pixys, read window.
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(0, bytes.length, I2cDeviceSynch.ReadMode.ONLY_ONCE);
        this.deviceClient.setReadWindow(readWindow);

        ///now we read through the bytes
        bytes = this.deviceClient.read(readWindow.getRegisterFirst(), readWindow.getRegisterCount());


        int index = 0;
        for (; index < bytes.length - 1; ++index) {
            int b1 = bytes[index];
            if (b1 < 0)
                b1 += 256;

            int b2 = bytes[index + 1];
            if (b2 < 0)
                b2 += 256;

            if (b1 == 0x55 && b2 == 0xaa)
                break;
        }

        if (index == 63)
            return null;
        else if (index == 0)
            index += 2;


        for (int byteOffset = index; byteOffset < bytes.length - BLOCK_SIZE - 1; ) {
            // checking for sync block
            int b1 = bytes[byteOffset];
            if (b1 < 0)
                b1 += 256;

            int b2 = bytes[byteOffset + 1];
            if (b2 < 0)
                b2 += 256;

            if (b1 == 0x55 && b2 == 0xaa) {
                // copy block into temp buffer
                byte[] temp = new byte[BLOCK_SIZE];
                StringBuilder sb = new StringBuilder("Data : ");
                for (int tempOffset = 0; tempOffset < BLOCK_SIZE; ++tempOffset) {
                    temp[tempOffset] = bytes[byteOffset + tempOffset];
                    sb.append(temp[tempOffset] + ", ");
                }

                PixyBlock block = bytesToBlock(temp);

                //Added so blocks are only added if their signature is 1 to remove noise from signal
                if (block.signature == 1) {
                    detectedBlocks.add(block);
                    byteOffset += BLOCK_SIZE - 1;
                } else
                    ++byteOffset;
            } else
                ++byteOffset;
        }


        if (detectedBlocks.size() >= 2) {
            PixyBlock leftBlock;
            PixyBlock rightBlock;
            if (detectedBlocks.get(0).centerX > detectedBlocks.get(1).centerX) {
                leftBlock = detectedBlocks.get(1);
                rightBlock = detectedBlocks.get(0);
            } else {
                leftBlock = detectedBlocks.get(0);
                rightBlock = detectedBlocks.get(1);
            }
//            double difference = (double) (rightBlock.centerX + leftBlock.centerX) / 2;
//            setLastOffset(difference);
//            double total = (rightBlock.centerX) - (leftBlock.centerX);
//            getDistance(total, difference);


//        else{
//
//            setLastOffset(160); //Keeps robot going straight if only one signal is picked up
//

//    } else{
//        setLastOffset(160); //Keeps robot going straight if nothing is picked up
//        setInRange(false);
//    }

        }
        return detectedBlocks;
    }

    public PixyBlock bytesToBlock(@NonNull byte[] bytes) {
        PixyBlock pixyBlock = new PixyBlock();
        pixyBlock.sync = bytesToInt(bytes[1], bytes[0]);
        pixyBlock.checksum = bytesToInt(bytes[3], bytes[2]);


        pixyBlock.signature = orBytes(bytes[5], bytes[4]);
        pixyBlock.centerX = ((((int) bytes[7] & 0xff) << 8) | ((int) bytes[6] & 0xff));
        pixyBlock.centerY = ((((int) bytes[9] & 0xff) << 8) | ((int) bytes[8] & 0xff));
        pixyBlock.width = ((((int) bytes[11] & 0xff) << 8) | ((int) bytes[10] & 0xff));
        pixyBlock.height = ((((int) bytes[13] & 0xff) << 8) | ((int) bytes[12] & 0xff));
        return pixyBlock;
    }

    public int bytesToInt(int b1, int b2) {
        if (b1 < 0)
            b1 += 256;

        if (b2 < 0)
            b2 += 256;

        int intValue = b1 * 256;
        intValue += b2;
        return intValue;
    }

    public int orBytes(byte b1, byte b2) {
        return (b1 & 0xff) | (b2 & 0xff);
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "pixy";
    }

    @Override
    protected boolean internalInitialize(@NonNull PixyCam.PixyCamParams pixyCamParams) {
        this.parameters = pixyCamParams.clone();
        deviceClient.setI2cAddress(parameters.i2cAddr);
        return true;
    }
}
