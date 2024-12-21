package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers.ThreeWheelLocalizer;

@TeleOp
@Config
public class Teleo extends LinearOpMode {
    private PoseUpdater poseUpdater;

    private DcMotorEx deadWheel;

    @Override
    public void runOpMode() throws InterruptedException {
        ThreeWheelLocalizer localizer = new ThreeWheelLocalizer(hardwareMap);

        deadWheel = hardwareMap.get(DcMotorEx.class, "par1");

        waitForStart();
        while(opModeIsActive()){
            poseUpdater = new PoseUpdater(hardwareMap, localizer);
            poseUpdater.update();
            telemetry.addData("Encoderex lives?", deadWheel.getVelocity());

            if(poseUpdater == null){
                telemetry.addLine("null localizer");
            } else {
                telemetry.addLine("not null localizer");
            }

            if(poseUpdater.getPose().equals(null)){
                telemetry.addLine("Null x");
            } else {
                telemetry.addLine("not .equals null");
            }

            telemetry.addData("acceleration", poseUpdater.getAcceleration().getMagnitude());
            telemetry.addData("X", poseUpdater.getPose().getX());
            poseUpdater.update();
            telemetry.update();
        }
    }
}
