package org.firstinspires.ftc.teamcode.nordicStorm.actionClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class LLActions extends DriveActions{

    private Limelight3A limelight3A;
    public LLActions(HardwareMap hardwareMap, Pose2d startingPos, Gamepad gamepad1){
        super(hardwareMap, startingPos, gamepad1);
        limelight3A = hardwareMap.get(Limelight3A.class, "lime");

    }
    public Action chaseThing(){
        return new ChaseThing();
    }

    public class ChaseThing implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

}
