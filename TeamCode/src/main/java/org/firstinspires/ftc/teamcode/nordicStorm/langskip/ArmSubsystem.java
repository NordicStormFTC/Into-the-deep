package org.firstinspires.ftc.teamcode.nordicStorm.langskip;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.lang.reflect.Method;

/**
 The thought here is that the vision subsystem will produce data for the
 hardware to react to. Hence the vision subsystem extends the main robot class 'Langskip'
 and the arm subsystem inherits the data produced by the vision subsystem. The main
 robot class carries only the subsystems to be used in opmodes, and holds no information
 besides one object of each subsystem.
 */
public class ArmSubsystem extends VisionSubsystem {

    private final DcMotor arm;

    private final Servo grabby;

    private final Servo elbow;

    /**
     * using a package private constructor to
     * maintain immutability.
     * @param hardwareMap publicly instantiate me in Langskip!
     */

     ArmSubsystem(HardwareMap hardwareMap) {
        super(hardwareMap);
        arm = hardwareMap.get(DcMotor.class, "arm");
        grabby = hardwareMap.get(Servo.class, "grabby");
        elbow = hardwareMap.get(Servo.class, "elbow");
    }

    public void setArmUp(double target) {
        final double posStamp = arm.getCurrentPosition();

        double error = target - posStamp;
        double dError = (arm.getCurrentPosition() - posStamp) / 0.1;

        double power = (1 * error) + (1 * dError);
        arm.setPower(power);
    }

    public double getArmPosition(){
         return arm.getCurrentPosition();
    }

    public void setArmPower(double power){
         arm.setPower(power);
    }
}
