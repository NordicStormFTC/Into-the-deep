package org.firstinspires.ftc.teamcode.nordicStorm;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class VisionSubSystem extends DriveTrain {
    protected final PixyCam pixyCam;
    protected final Limelight3A limelight;
    private LLResult results;

    public VisionSubSystem(HardwareMap hardwareMap) {
        super(hardwareMap);
        pixyCam = hardwareMap.get(PixyCam.class, "pixy");
        limelight = hardwareMap.get(Limelight3A.class, "limeLight");
    }

    public static class PixyActions extends VisionSubSystem {

        public PixyActions(HardwareMap hardwareMap) {
            super(hardwareMap);
        }

        public Action alignWithPiece() {
            return new AlignWithPiece();
        }

        public class AlignWithPiece implements Action {

            List<PixyBlock> blockList = new ArrayList<PixyBlock>();
            ElapsedTime dTimer = new ElapsedTime();
            ElapsedTime timer = new ElapsedTime();

            double lastError;
            double lastTime;

            private final double tolerance = 1;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                blockList = pixyCam.read();

                if(dTimer.milliseconds() > 0.1){
                    lastTime = dTimer.milliseconds();
                    dTimer.reset();
                }

                if(!blockList.isEmpty()){

                   double error = blockList.get(0).centerX;
                }

//
//                double kP = 0.1;
//                double kD = 0.1;
//                double p = kP * error;
//                double d = kD * (error - lastError)/ (dTimer.milliseconds() - lastTime);

                //double power = p + d;

               // driveBase.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),power));
                //return Math.abs(error) < tolerance && timer.milliseconds() > 500;
                return true;
            }
        }
    }

    public static class LimeLightActions extends VisionSubSystem {

        public LimeLightActions(HardwareMap hardwareMap) {
            super(hardwareMap);
        }

        public Action alignWithPiece() {
            return new AlignWithPiece();
        }

        public class AlignWithPiece implements Action {

            ElapsedTime dTimer = new ElapsedTime();
            ElapsedTime timer = new ElapsedTime();

            double lastError;
            double lastTime;

            private final double tolerance = 1;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                LLResult results = limelight.getLatestResult();

                if(dTimer.milliseconds() > 0.1){
                    lastError = results.getTx();
                    lastTime = dTimer.milliseconds();
                    dTimer.reset();
                }

                double kP = 0.1;
                double kD = 0.1;

                double error = results.getTx();

                double p = kP * error;
                double d = kD * (error - lastError)/ (dTimer.milliseconds() - lastTime);

                double power = p + d;

                driveBase.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),power));
                return Math.abs(error) < tolerance && timer.milliseconds() > 500;
            }
        }
    }
}
