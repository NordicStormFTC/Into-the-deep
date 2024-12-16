package org.firstinspires.ftc.teamcode.nordicStorm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem {

    private final int UP = 2000;
    private final int down = 0;
    private final DcMotor arm;

    public ArmSubsystem(HardwareMap hardwareMap){
        arm = hardwareMap.get(DcMotor.class, "arm");
    }

    public void setArmUp(double target){
        double error = target - arm.getCurrentPosition();
        double power = 1 * error;
        arm.setPower(power);
    }

}
