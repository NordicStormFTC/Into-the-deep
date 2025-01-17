package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nordicStorm.langskip.ArmSubsystem;
import org.firstinspires.ftc.teamcode.nordicStorm.langskip.Langskip;

@Config
@TeleOp
public class HardwareTesting extends LinearOpMode {


    public static double servoPos;
    public static double armPos;

    public static boolean ARM_UP;
    public static boolean ARM_DOWN;
    public static boolean GRIP;

    @Override
    public void runOpMode() throws InterruptedException {

        Langskip langskip = new Langskip(hardwareMap);

        langskip.follower.startTeleopDrive();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {

            langskip.armSubsystem.wrist.setPosition(ArmSubsystem.ArmConstants.WRIST_DOWN);
            //langskip.visionSubsystem.setLimelightDriveController();
//            if (gamepad1.a) {
//                langskip.armSubsystem.elbow.setPosition(ArmSubsystem.ArmConstants.ELBOW_UP);
//            }
//            if (gamepad1.b) {
//                langskip.armSubsystem.elbow.setPosition(ArmSubsystem.ArmConstants.ELBOW_DOWN);
//            }
//            if (gamepad1.right_trigger > .5) {
//                langskip.armSubsystem.setTarget(90);
//            }
//            if (gamepad1.left_trigger > .5) {
//                langskip.armSubsystem.setTarget(180);
//            }
//            if (gamepad2.a) {
//                langskip.armSubsystem.gripper.setPosition(ArmSubsystem.ArmConstants.GRIPPER_OPEN);
//            }
//            if (gamepad2.b) {
//                langskip.armSubsystem.gripper.setPosition(ArmSubsystem.ArmConstants.GRIPPER_CLOSE);
//            }

            // right trigger open left trigger close
            //a down and out
            //y up/folded
            //x low basket

//
//            langskip.armSubsystem.runArm(telemetry);

            if(gamepad2.a){
                langskip.armSubsystem.putDownElbow();
            }
            if(gamepad2.right_trigger > 0.5){
                langskip.armSubsystem.openGripper();
            }
            if(gamepad2.left_trigger > 0.5){
                langskip.armSubsystem.closeGripper();
            }
            if(gamepad2.y){
                langskip.armSubsystem.foldInElbow();
            }
            if(gamepad2.x){
                langskip.armSubsystem.setScoringPos(ArmSubsystem.ScoringPosition.LOW_BASKET);
            }
            if(gamepad1.left_trigger > 0.5){
                langskip.visionSubsystem.seeknDestroy(langskip.follower, telemetry);
            } else {
                langskip.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }

            langskip.follower.update();

            telemetry.update();
        }
    }
}
