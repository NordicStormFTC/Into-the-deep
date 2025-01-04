package org.firstinspires.ftc.teamcode.nordicStorm.alexsGoatedi2cPackage;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

public class GoatedI2cDeviceWrapper {

    private final Class<GoatedI2cDeviceSkeleton> device;
    private final HardwareMap hardwareMap;

    public GoatedI2cDeviceWrapper(final HardwareMap hardwareMap, final Class<GoatedI2cDeviceSkeleton> device) {
        this.device = device;
        this.hardwareMap = hardwareMap;
    }



}
