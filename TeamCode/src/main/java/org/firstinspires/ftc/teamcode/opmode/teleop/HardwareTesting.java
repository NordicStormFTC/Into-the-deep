package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.nordicStorm.alexsGoatedi2cPackage.GoatedI2cDeviceWrapper;
import org.firstinspires.ftc.teamcode.nordicStorm.langskip.ArmSubsystem;
import org.firstinspires.ftc.teamcode.nordicStorm.langskip.Langskip;
import org.firstinspires.ftc.teamcode.nordicStorm.pixy.I2C;
import org.firstinspires.ftc.teamcode.nordicStorm.pixy.Pixy3;
import org.firstinspires.ftc.teamcode.nordicStorm.pixy.PixyBlock;
import org.firstinspires.ftc.teamcode.nordicStorm.pixy.PixyCam;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp
public class HardwareTesting extends LinearOpMode {

//    GoatedI2cDeviceWrapper wrapper;
//    I2C i2C;
//    Langskip langskip;

    Pixy3 pixy3;
//    List<PixyBlock> blocks = new ArrayList<>();
   // PixyBlock block;
    //Servo servo;
    public static double servoPos;
    @Override
    public void runOpMode() throws InterruptedException {
//        wrapper = new GoatedI2cDeviceWrapper(hardwareMap, I2C.class);


       // servo = hardwareMap.get(Servo.class, "elbow");
        pixy3 = hardwareMap.get(Pixy3.class, "pixy3");
        pixy3.requestSync();

        // langskip = new Langskip(hardwareMap);
//        langskip.follower.setStartingPose(langskip.startPose);
//        langskip.follower.startTeleopDrive();
       // langskip.armSubsystem.setTargetTicks(90);
       // langskip.armSubsystem.offset = langskip.armSubsystem.getAbsoluteArmTicks();
        waitForStart();
        while (opModeIsActive()) {
            pixy3.readSync(telemetry);
            //langskip.armSubsystem.runArm(telemetry);
            //close .5
            //servo.setPosition(.45);
//            if(gamepad1.right_trigger > 0.6){
//
//            } else if(gamepad1.right_trigger < 0.6){
//                servo.setPosition(.5);
//            }
            //pixy3.updateBlock(telemetry);
          // block = pixy3.getBlock(telemetry);
           //telemetry.addData("X", blocks.get(1).centerX);
//            langskip.armSubsystem.elbow.setPosition(servoPos);
//
//            telemetry.addData("Target in ticks", langskip.armSubsystem.targetTicks);
//            telemetry.addData("current angle", langskip.armSubsystem.getArmAngle());
//            telemetry.addData("Ticks", langskip.armSubsystem.getArmPositionTicks());
//            telemetry.addData("Ticks converted to degrees", langskip.armSubsystem.getArmPositionTicks() * ArmSubsystem.ArmConstants.TICKS_TO_DEGREES);
            telemetry.update();
        }
    }
}
