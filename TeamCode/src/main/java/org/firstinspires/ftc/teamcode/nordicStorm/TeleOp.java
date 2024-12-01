package org.firstinspires.ftc.teamcode.nordicStorm;

import android.graphics.Canvas;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
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
    private DriveTrain driveTrain;
//    private Telemetry dashTelemetry = dash.getTelemetry();

//    private List<Action> runningActions = new ArrayList<>();


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dash.updateConfig();
        TelemetryPacket p = new TelemetryPacket();

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
           // driveTrain = new DriveTrain(hardwareMap, new Pose2d(0, 0, 0), gamepad1, gamepad2);
            p.put("hello", 2);
            dash.sendTelemetryPacket(p);


            // Java bitmap class/covert pixydata to botmap
            //  dash.sendImage();


//            Action traj = driveTrain.driveBase.actionBuilder(driveTrain.startingPos)
//                    //.stopAndAdd(driveTrain.driveActions.gamepadDrive())
//                    .lineToXConstantHeading(10)
//                    .build();
//
            // Actions.runBlocking(new InstantAction(() -> driveTrain.driveBase.leftFront.setPower(1));
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

    public class TeleOpAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }
}
