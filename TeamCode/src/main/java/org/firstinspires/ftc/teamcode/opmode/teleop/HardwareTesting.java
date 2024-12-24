package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

@Config
@TeleOp
public class HardwareTesting extends LinearOpMode {

    DcMotor motor;
    DcMotor drive;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "strafe");
        drive = hardwareMap.get(DcMotor.class, "par1");
        waitForStart();
        while(opModeIsActive()){
          telemetry.addData("strafe", motor.getCurrentPosition());
            telemetry.addData("drive", drive.getCurrentPosition());

            telemetry.addData("a", gamepad1.a)
                    ;
            telemetry.update();
        }
    }
}
