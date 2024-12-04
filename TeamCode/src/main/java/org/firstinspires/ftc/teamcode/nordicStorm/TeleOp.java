package org.firstinspires.ftc.teamcode.nordicStorm;

import android.graphics.Canvas;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.nordicStorm.actionClasses.DriveActions;
import org.firstinspires.ftc.teamcode.nordicStorm.actionClasses.LLActions;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {


    // private final VisionSubSystem.PixyActions pixy = driveTrain.getPixyActions();
    //private final VisionSubSystem.LimeLightActions limeLight = driveTrain.getLimeLightActions();

    private FtcDashboard dash = FtcDashboard.getInstance();
    private DriveActions drive;
    private final Pose2d startingPos = new Pose2d(0,0,0);
    private LLActions llActions;
    private List<Action> runningActions = new ArrayList<>();
    private boolean doGpDrive = true;
   // private Arm arm;
    //    private Telemetry dashTelemetry = dash.getTelemetry();

//    private List<Action> runningActions = new ArrayList<>();


    @Override
    public void runOpMode() {
        drive = new DriveActions(hardwareMap,startingPos,gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dash.updateConfig();

        llActions = new LLActions(hardwareMap, startingPos, gamepad1);
        TelemetryPacket p = new TelemetryPacket();

        MecanumKinematics kin = drive.driveBase.kinematics;
        Localizer loc = drive.driveBase.localizer;
        Twist2d twist = loc.update().value();
        Pose2d pos = new Pose2d(twist.line, twist.angle);

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
           // driveTrain = new DriveTrain(hardwareMap, new Pose2d(0, 0, 0), gamepad1, gamepad2);
            p.put("pos", drive.driveBase.updatePoseEstimate().component1());
            dash.sendTelemetryPacket(p);
            dash.updateConfig();
            telemetry.update();

            if(doGpDrive){
                runningActions.add(drive.gamepadDrive());
            }
            if(gamepad1.a){
                doGpDrive = false;
                runningActions.add(llActions.chaseThing());

            } else if(!gamepad1.a){
                doGpDrive = true;
            }


            List<Action> newActions = new ArrayList<>();
            for(Action action : runningActions){
                if(action.run(new TelemetryPacket())){
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            // Java bitmap class/covert pixydata to botmap
            //  dash.sendImage();


//            Action traj = driveTrain.driveBase.actionBuilder(driveTrain.startingPos)
//                    //.stopAndAdd(driveTrain.driveActions.gamepadDrive())
//                    .lineToXConstantHeading(10)
//                    .build();
//
        }
//        // updated based on gamepads
//
//
//        // update running actions
//        List<Action> newActions = new ArrayList<>();
//        for (Action action : runningActions) {
//            action.preview(packet.fieldOverlay());
//            if (action.run(packet)) {
//                newActions.add(action);
//            }
//        }
//        runningActions = newActions;
//
//        dash.sendTelemetryPacket(packet);

    }
}
