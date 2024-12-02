package org.firstinspires.ftc.teamcode.nordicStorm.actionClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LLActions extends DriveActions{

    public LLActions(HardwareMap hardwareMap, Pose2d startingPos, Gamepad gamepad1){
        super(hardwareMap, startingPos, gamepad1);

    }
    public Action chaseThing(){
        return new ChaseThing();
    }

    public class ChaseThing implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            driveBase.leftFront.setPower(1);
            return false;
        }
    }

}
