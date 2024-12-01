package org.firstinspires.ftc.teamcode.nordicStorm;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class DriveTrain {

//    public final VisionSubSystem.LimeLightActions llActions;
//    public final VisionSubSystem.PixyActions pixyActions;
    public final MecanumDrive driveBase;
   // public final DriveTrainActions driveActions;
    public final Arm.ArmActions armActions;

    protected Gamepad gamepad1;
    protected Gamepad gamepad2;

    Pose2d startingPos;
    Pose2d currentPos;

    public DriveTrain(HardwareMap hardwareMap, Pose2d startingPos, Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.startingPos = startingPos;
        driveBase = new MecanumDrive(hardwareMap, startingPos);
//        pixyActions = new VisionSubSystem.PixyActions(hardwareMap);
//        llActions = new VisionSubSystem.LimeLightActions(hardwareMap);
        //driveActions = new DriveTrainActions(hardwareMap, startingPos, gamepad1, gamepad2);
        armActions = new Arm.ArmActions(hardwareMap, startingPos, gamepad1, gamepad2);
    }

    public DriveTrain(HardwareMap hardwareMap) {
//        pixyActions = new VisionSubSystem.PixyActions(hardwareMap);
//        llActions = new VisionSubSystem.LimeLightActions(hardwareMap);
       // driveActions = new DriveTrainActions(hardwareMap, startingPos, gamepad1, gamepad2);
        driveBase = new MecanumDrive(hardwareMap, startingPos);
        armActions = new Arm.ArmActions(hardwareMap, startingPos, gamepad1, gamepad2);
    }

//    public VisionSubSystem.PixyActions getPixyActions() {
//        return pixyActions;
//    }
//
//    public VisionSubSystem.LimeLightActions getLimeLightActions() {
//        return llActions;
//    }

//    public DriveTrainActions getDriveActions() {
//        return driveActions;
//    }

    public Arm.ArmActions getArmActions(){
        return armActions;
    }


//    public static class DriveTrainActions extends DriveTrain {
//
//        public DriveTrainActions(HardwareMap hardwareMap, Pose2d startingPos, Gamepad gamePad1, Gamepad gamePad2) {
//            super(hardwareMap, startingPos, gamePad1, gamePad2);
//
//        }
//
//        public Action gamepadDrive() {
//            return new GamepadDrive();
//        }
//
//        public Action driveForInches(double yTarget, double targetHeading){
//            return new DriveForInches(yTarget, targetHeading);
//        }
//
//        public class GamepadDrive implements Action {
//
////            Vector2d xyVector;
////            PoseVelocity2d poseVelocity2d;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                //xyVector = new Vector2d(gamepad1.left_stick_x, gamepad1.left_stick_y);
//                //driveBase.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 1), 0));
//                driveBase.leftFront.setPower(1);
//                return false;
//            }
//        }
//
//        public class DriveForInches implements Action {
//
//            private final double yTarget;
//            private final double targetHeading;
//
//            public DriveForInches(double yTarget, double targetHeading){
//                this.targetHeading = targetHeading;
//                this.yTarget = yTarget;
//            }
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//               // MecanumDrive.TurnAction
//                return false;
//            }
//
//            public void cancel(){
//
//            }
//        }
//    }
}
