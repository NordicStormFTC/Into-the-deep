package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nordicStorm.langskip.Langskip;

@Config
@TeleOp
public class HardwareTesting extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Langskip langskip = new Langskip(hardwareMap);
        langskip.driveTrain.follower.startTeleopDrive();
        waitForStart();

        while (opModeIsActive()) {

//            if (gamepad2.y) {
//                langskip.pixyArm.setArm(5);
//            }
//            if (gamepad2.a) {
//                langskip.pixyArm.setArm(4);
//            }
//            if(gamepad2.right_trigger > 0.5){
//                langskip.pixyArm.openGripper();
//            }
//            if(gamepad2.left_trigger > 0.5){
//                langskip.pixyArm.closeGripper();
//            }
            if(gamepad1.right_trigger > 0.5){
                langskip.driveTrain.seeknDestroy(telemetry);
            }
            langskip.driveTrain.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
            langskip.driveTrain.follower.update();
            langskip.pixyArm.runArm(telemetry);
            langskip.pixyArm.runWrist(telemetry);langskip.pixyArm.runElbow(telemetry);
            telemetry.update();
        }
    }
}
