package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

@Config
@TeleOp
public class HardwareTesting extends LinearOpMode {

    Servo servo;

    public static double position;
    ServoController controller;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "wrist");
        waitForStart();
        while(opModeIsActive()){
            servo.setPosition(position);
            telemetry.update();
        }
    }
}
