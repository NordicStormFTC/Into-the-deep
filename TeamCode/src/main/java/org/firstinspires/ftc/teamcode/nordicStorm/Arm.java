package org.firstinspires.ftc.teamcode.nordicStorm;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    private DcMotorEx pendulum;

    public Arm(HardwareMap hardwareMap) {
    }

//    public Arm(HardwareMap hardwareMap, Pose2d startingPos, Gamepad gamepad1, Gamepad gamepad2) {
//        super(hardwareMap, startingPos, gamepad1, gamepad2);
  //  }

//    public static class ArmActions extends Arm {
//
//
////        public ArmActions(HardwareMap hardwareMap, Pose2d startingPos, Gamepad gamepad1, Gamepad gamepad2) {
////            super(hardwareMap, startingPos, gamepad1, gamepad2);
////        }
//
//        public Action doArm(){
//            return new DoArm();
//        }
//
//
//        public class DoArm implements Action {
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                return false;
//            }
//        }
//    }
}
