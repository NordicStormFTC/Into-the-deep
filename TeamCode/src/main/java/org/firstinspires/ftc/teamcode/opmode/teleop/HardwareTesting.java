package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nordicStorm.langskip.ArmSubsystem;
import org.firstinspires.ftc.teamcode.nordicStorm.langskip.Langskip;

@Config
@TeleOp
public class HardwareTesting extends LinearOpMode {

    Langskip langskip;

    public static double servoPos;
    @Override
    public void runOpMode() throws InterruptedException {

        langskip = new Langskip(hardwareMap);
        langskip.follower.setStartingPose(langskip.startPose);
        langskip.follower.startTeleopDrive();
        langskip.armSubsystem.setTargetTicks(90);
        langskip.armSubsystem.offset = langskip.armSubsystem.getAbsoluteArmTicks();
        waitForStart();
        while (opModeIsActive()) {
            langskip.armSubsystem.elbow.setPosition(servoPos);

            telemetry.addData("Target in ticks", langskip.armSubsystem.targetTicks);
            telemetry.addData("current angle", langskip.armSubsystem.getArmAngle());
            telemetry.addData("Ticks", langskip.armSubsystem.getArmPositionTicks());
            telemetry.addData("Ticks converted to degrees", langskip.armSubsystem.getArmPositionTicks() * ArmSubsystem.ArmConstants.TICKS_TO_DEGREES);
            telemetry.update();
        }
    }
}
