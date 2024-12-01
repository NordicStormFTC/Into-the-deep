package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.nordicStorm.DriveTrain;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class OP extends OpMode {
    FtcDashboard dash = FtcDashboard.getInstance();

    private List<Action> runningActions = new ArrayList<>();
    private Robot robot;

    private DriveTrain driveTrain;
    private MecanumDrive drive;
    @Override
    public void init() {
        driveTrain = new DriveTrain(hardwareMap);
        drive = driveTrain.driveBase;
        dash.onOpModePreStart(this);
    }

    @Override
    public void loop() {
        drive.updatePoseEstimate();
        TelemetryPacket packet = new TelemetryPacket();
        if(dash.isEnabled()){
            packet.addLine("hello");
            dash.sendTelemetryPacket(packet);
        }
        List<Action> newActions = new ArrayList<>();
        for(Action action : newActions){
            action.preview(packet.fieldOverlay());
            if(action.run(packet)){
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
        drive.leftBack.setPower(1);
    }

    @Override
    public void stop(){
        dash.onOpModePostStop(this);
    }

}
