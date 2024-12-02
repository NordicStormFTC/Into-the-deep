package org.firstinspires.ftc.teamcode.nordicStorm.actionClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class DriveActions {
    private final Pose2d startingPos;
    public final MecanumDrive driveBase;
    private final Gamepad gamepad1;
    public DriveActions(HardwareMap hardwareMap, Pose2d startingPos, Gamepad gamepad1) {
        this.startingPos = startingPos;
        this.gamepad1 = gamepad1;
        driveBase = new MecanumDrive(hardwareMap, startingPos);
    }
    public Action gamepadDrive() {
        return new GamepadDrive();
    }

    public class GamepadDrive implements Action {

//            Vector2d xyVector;
//            PoseVelocity2d poseVelocity2d;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Vector2d xy = new Vector2d(gamepad1.left_stick_x,gamepad1.left_stick_y);
            //xyVector = new Vector2d(gamepad1.left_stick_x, gamepad1.left_stick_y);
            driveBase.setDrivePowers(new PoseVelocity2d(xy,gamepad1.right_stick_x));
            return false;
        }
    }
}
