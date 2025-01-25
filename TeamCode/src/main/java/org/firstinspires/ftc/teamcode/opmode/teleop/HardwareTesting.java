package org.firstinspires.ftc.teamcode.opmode.teleop;

import android.graphics.Canvas;
import android.graphics.Picture;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nordicStorm.pixy.EmptyBlock;
import org.firstinspires.ftc.teamcode.nordicStorm.pixy.Pixy;
import org.firstinspires.ftc.teamcode.nordicStorm.pixy.PixyBlock;

@Config
@TeleOp
public class HardwareTesting extends LinearOpMode {

    Pixy pixy;

    @Override
    public void runOpMode() throws InterruptedException {

        pixy = hardwareMap.get(Pixy.class, "pixy");
        //Langskip langskip = new Langskip(hardwareMap);

//        langskip.follower.startTeleopDrive();
//        langskip.follower.setStartingPose(new Pose(0,0));

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        pixy.turnOnLamps();
        PixyBlock block;
        waitForStart();

        Canvas canvas;
        while (opModeIsActive()) {
           block = pixy.getBlock();


            telemetry.addLine(block.toString());

           // langskip.armSubsystem.wrist.setPosition(ArmSubsystem.ArmConstants.WRIST_DOWN);
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

//            if(gamepad2.a){
//                langskip.armSubsystem.putDownElbow();
//                //langskip.armSubsystem.wrist.setPosition(0);
//            }
//            if(gamepad2.b){
//                langskip.armSubsystem.putDownArm();
//            }
//            if(gamepad2.right_trigger > 0.5){
//                langskip.armSubsystem.openGripper();
//               // langskip.armSubsystem.wrist.setPosition(0.5);
//            }
//            if(gamepad2.left_trigger > 0.5){
//                langskip.armSubsystem.closeGripper();
//                //langskip.armSubsystem.wrist.setPosition(.55);
//            }
//            if(gamepad2.y){
//                langskip.armSubsystem.foldInElbow();
//            }
//            if(gamepad2.x){
//                langskip.armSubsystem.setScoringPos(ArmSubsystem.ScoringPosition.LOW_BASKET);
//            }
//            if(gamepad1.left_trigger > 0.5){
//                langskip.visionSubsystem.seeknDestroy(langskip.follower, telemetry);
//            } else {
//                langskip.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
//            }
//            langskip.armSubsystem.runArm(telemetry);
//
//            langskip.follower.update();

            telemetry.update();
        }
    }
}
