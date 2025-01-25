package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ThreadPool;

@Autonomous
public class TheAuto extends LinearOpMode {

    DcMotor encoder;

    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;


    public double offset;

    @Override
    public void runOpMode() throws InterruptedException {


        encoder = hardwareMap.get(DcMotor.class, "par1");
        encoder.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft = hardwareMap.get(DcMotor.class, "front left");
        frontright = hardwareMap.get(DcMotor.class, "front right");
        backleft = hardwareMap.get(DcMotor.class, "back left");
        backright = hardwareMap.get(DcMotor.class, "back right");
        offset = encoder.getCurrentPosition();

        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {

            distence();
        }
    }

    public void weels(double power) {
        frontright.setPower(power);
        frontleft.setPower(power);
        backright.setPower(power);
        backleft.setPower(power);
    }

    void distence() {

        if (getPosition() < 2300) {
        weels(1);
        }
        else {
            weels(0 );
        }
    }

    public double getPosition() {
        return encoder.getCurrentPosition() - offset;
    }

}
