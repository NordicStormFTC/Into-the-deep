package org.firstinspires.ftc.teamcode.nordicStorm.langskip;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nordicStorm.pixy.PixyBlock;
import org.firstinspires.ftc.teamcode.nordicStorm.pixy.PixyCam;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;

/**
 * this class holds drive controllers in respect to camera results.
 * I repeat, the vision subsystem holds power to move the robot in reaction to what it sees.
 * The arm Subsystem will extend the vision subsystem, as the Pixy cam is located at the very end of the arm.
 */
public class VisionSubsystem {

    private final Limelight3A limeLight;

    //private final PixyCam pixy;

    private LLResult llResult;

    private List<PixyBlock> pixyData = new ArrayList<>();

    public final PIDFController limelightDriveController;
    private final PIDFController limelightRotationController;

    /**
     * using a package private constructor to
     * maintain immutability.
     *
     * @param hardwareMap publicly instantiate me in Langskip!
     */

    VisionSubsystem(@NonNull final HardwareMap hardwareMap) {
        limeLight = hardwareMap.get(Limelight3A.class, "lime");

        // pixy = hardwareMap.get(PixyCam.class, "pixy");

        limeLight.pipelineSwitch(0);

        limeLight.start();

        limelightDriveController = new PIDFController(new CustomPIDFCoefficients(0.03, 0, 0.03, 0.001));
        limelightRotationController = new PIDFController(new CustomPIDFCoefficients(0.03, 0, 0.035, 0.001));

        limelightDriveController.setTargetPosition(-15);
        limelightRotationController.setTargetPosition(0);
    }

    /**
     * call this method in the loop of your opMode!
     */
    public void updateCameras() {
        llResult = limeLight.getLatestResult();
        //pixyData = pixy.read();
    }

    public LLResult getLLResult() {
        updateCameras();
        return llResult;
    }

    public List<PixyBlock> getPixyResults() {
        updateCameras();
        return pixyData;
    }

    /**
     * make sure that when you call this in the teleOp you arnt trying to also call any sort of gamepad drive
     * this function will move the robot of its own accord
     */

    public void seeknDestroy(@NonNull Follower follower, Telemetry telemetry) {
        if (getLLResult().isValid() && getLLResult().getStaleness() < 100) {
            double rotationError = llResult.getTx();
            double driveError = llResult.getTy();

            limelightRotationController.updatePosition(rotationError);
            limelightDriveController.updatePosition(driveError);

            telemetry.addLine("Charging");
            telemetry.addData("tx", rotationError);
            telemetry.addData("Ty", driveError);

            double rotationPower = 0;
            double drivePower = 0;

            /*
             * if the rotation error is within [-1, 1] we intentionally don't apply power
             */
            if (Math.abs(limelightRotationController.getError()) > 1) {
                rotationPower = limelightRotationController.runPIDF();
            }


            if (Math.abs(limelightDriveController.getError()) > 0.5) {
                drivePower = limelightDriveController.runPIDF();
            }

            telemetry.addData("Drive PID", limelightDriveController.getError());

            follower.setTeleOpMovementVectors(-drivePower, 0, rotationPower, true);
        } else {
            telemetry.addLine("No valid results!");
        }
        telemetry.update();
    }

}
