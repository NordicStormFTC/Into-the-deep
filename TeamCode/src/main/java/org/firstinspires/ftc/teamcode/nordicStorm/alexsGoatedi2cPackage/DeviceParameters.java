package org.firstinspires.ftc.teamcode.nordicStorm.alexsGoatedi2cPackage;

import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.GoBildaPinpointDriver;


public class DeviceParameters implements Cloneable {
    I2cAddr i2cAddr;

    public DeviceParameters(I2cAddr defaultAddress){
        this.i2cAddr = defaultAddress;
    }

    public DeviceParameters clone() {
        try {
            return (DeviceParameters) super.clone();
        } catch (CloneNotSupportedException e) {
            throw new RuntimeException("Internal error: parameters not cloneable");
        }
    }
}
