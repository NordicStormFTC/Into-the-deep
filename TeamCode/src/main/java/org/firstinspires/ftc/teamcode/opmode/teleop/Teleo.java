package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.nordicStorm.langskip.Langskip;

@Config
@TeleOp
public class Teleo extends LinearOpMode {
    private Langskip langskip;

    public static double elbow;
    public static double wrist;
    public static double arm;

    private Servo servo;
    @Override
    public void runOpMode() throws InterruptedException {
        langskip = new Langskip(this.hardwareMap);

        boolean doGPDrive = true;

        langskip.visionSubsystem.setLimelightDriveController(0.001,0,0.1,0.001);
        langskip.visionSubsystem.setLimelightRotationController(0.001,0,0.1,0.001);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servo = hardwareMap.get(Servo.class, "elbow");
        waitForStart();
        while (opModeIsActive()) {
            langskip.armSubsystem.setArmPIDF(0.001,0,0.1,0.001);

//            langskip.armSubsystem.elbow.setPosition(elbow);
//            langskip.armSubsystem.wrist.setPosition(wrist);

           // telemetry.addData("elboq", servo.getPosition());
//            telemetry.addData("wrist", langskip.armSubsystem.wrist.getPosition());
//            telemetry.addData("elbow", langskip.armSubsystem.elbow.getPosition());

//            if (gamepad1.right_trigger > 0.5) {
//                doGPDrive = false;
//                langskip.visionSubsystem.seeknDestroy(langskip.follower);
//            } else if (gamepad1.right_trigger < 0.5) {
//                doGPDrive = true;
//            }


            langskip.follower.setTeleOpMovementVectors(1, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            telemetry.update();
            langskip.follower.update();
        }

    }
}
