package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.nordicStorm.langskip.Langskip;
import org.firstinspires.ftc.teamcode.nordicStorm.langskip.PixyArm;
import org.firstinspires.ftc.teamcode.nordicStorm.pixy.Pixy;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;

@TeleOp
public class PixyTEst extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

//        Pixy pixy2 = hardwareMap.get(Pixy.class, "pixy");
//        pixy2.turnOnLamps();


        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");
        Servo servo = hardwareMap.get(Servo.class, "servo");
        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(0.5);
            servo.setPosition(0.75);
            telemetry.update();
//            Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
//            Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
//            Drawing.sendPacket();
        }
    }
}
