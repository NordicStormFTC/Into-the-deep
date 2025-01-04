package org.firstinspires.ftc.teamcode.nordicStorm.pixy;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import java.util.ArrayList;


/**
 * @ annotations allow us to register Pixy2 as a device when configuring robot on driver hub.
 * note that MCP9808 is in fact the XML tag for the adafruit temperature I2C Sensor. I was not able to find
 * the appropriate tag for Pixy2 and used as shown in this demonstration 'writing an I2C Device Driver'
 * https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Writing-an-I2C-Driver
 **/

//@I2cDeviceType
//@DeviceProperties(name = "Pixy2 Smart Camera", description = "Pixy2 Smart Camera", xmlTag = "MCP9808")
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
        detectedBlocks.clear();
        //detectedBlocks = new ArrayList<>();

        byte[] bytes = new byte[26];

        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(0, bytes.length, I2cDeviceSynch.ReadMode.ONLY_ONCE);
        this.deviceClient.setReadWindow(readWindow);

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

                for (int tempOffset = 0; tempOffset < BLOCK_SIZE; ++tempOffset) {
                    temp[tempOffset] = bytes[byteOffset + tempOffset];
                }
                PixyBlock block = PixyUtils.bytesToBlock(temp);

                //Added so blocks are only added if their signature is 1 to remove noise from signal
                if (block.signature == 1) {
                    detectedBlocks.add(block);
                    byteOffset += BLOCK_SIZE - 1;
                } else
                    ++byteOffset;
            } else
                ++byteOffset;
        }

        return detectedBlocks;
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
