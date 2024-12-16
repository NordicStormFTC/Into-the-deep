package org.firstinspires.ftc.teamcode.nordicStorm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 The thought here is that the vision subsystem will produce data for the
 hardware to react to. Hence the vision subsystem extends the main robot class 'Langskip'
 and the arm subsystem inherits the data produced by the vision subsystem. The main
 robot class carries only the subsystems to be used in opmodes, and holds no information
 besides one object of each subsystem.
 */
public class ArmSubsystem extends VisionSubsystem {
    private final DcMotor arm;
    public ArmSubsystem(HardwareMap hardwareMap, OpType opType) {
        super(hardwareMap, opType);
        arm = hardwareMap.get(DcMotor.class, "arm");
    }
    public void setArmUp(double target) {
        final double posStamp = arm.getCurrentPosition();

        double error = target - posStamp;
        double dError = (arm.getCurrentPosition() - posStamp) / 0.1;

        double power = (1 * error) + (1 * dError);
        arm.setPower(power);
    }
}
