package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nordicStorm.langskip.Langskip;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;

@TeleOp
public class PixyTEst extends LinearOpMode {


    Langskip langskip;
    private DashboardPoseTracker dashboardPoseTracker;


    @Override
    public void runOpMode() throws InterruptedException {

        langskip = new Langskip(hardwareMap);

        Drawing.drawRobot(langskip.driveTrain.getPosition(telemetry), "#4CAF50");
        Drawing.sendPacket();

//        FtcDashboard dashboard = FtcDashboard.getInstance();
//
//
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//
//        Drawing.drawRobot(langskip.driveTrain.getPosition(telemetry), "#4CAF50");
//        Drawing.sendPacket();
        waitForStart();

        while (opModeIsActive()) {
            langskip.driveTrain.updatePosition(telemetry);
            telemetry.addData("x", langskip.driveTrain.getPosition(telemetry).getX());
            telemetry.addData("y", langskip.driveTrain.getPosition(telemetry).getY());

            telemetry.update();
//            Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
//            Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
//            Drawing.sendPacket();
        }
    }
}
