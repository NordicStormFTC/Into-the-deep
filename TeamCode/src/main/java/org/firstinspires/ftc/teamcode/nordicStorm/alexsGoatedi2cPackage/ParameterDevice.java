package org.firstinspires.ftc.teamcode.nordicStorm.alexsGoatedi2cPackage;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.teamcode.nordicStorm.pixy.Pixy3;

public class ParameterDevice extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynch, DeviceParameters> {

    public final I2cAddr DEFAULT_ADDRESS;

    public DeviceParameters parameters;

    public final String deviceName;

    protected ParameterDevice(final I2cDeviceSynch deviceSynch, final I2cAddr defaultAddress, DeviceParameters parameters, final String deviceName) {
        super(deviceSynch, true, parameters);

        this.parameters = parameters;

        DEFAULT_ADDRESS = defaultAddress;

        deviceClient.setI2cAddress(defaultAddress);

        this.deviceName = deviceName;

        registerArmingStateCallback(false);

        deviceClient.engage();
    }

    @Override
    protected boolean internalInitialize(@NonNull DeviceParameters deviceParameters) {
        parameters = deviceParameters.clone();
        deviceClient.setI2cAddress(parameters.i2cAddr);
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return deviceName;
    }
}
