package org.firstinspires.ftc.teamcode.nordicStorm.langskip;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

public class VisionSubsystem {

    private final Limelight3A limeLight;

    private LLResult results;

    private PIDFController limelightDriveController;
    private PIDFController limelightRotationController;

    /**
     * using a package private constructor to
     * maintain immutability.
     *
     * @param hardwareMap publicly instantiate me in Langskip!
     */

    VisionSubsystem(@NonNull HardwareMap hardwareMap) {
        limeLight = hardwareMap.get(Limelight3A.class, "lime");

        limeLight.pipelineSwitch(0);

        limeLight.start();

        limelightDriveController = new PIDFController(new CustomPIDFCoefficients(0.03, 0, 0.03, 0.001));
        limelightRotationController = new PIDFController(new CustomPIDFCoefficients(0, 0, 0, 0));

        limelightDriveController.setTargetPosition(0);
        limelightRotationController.setTargetPosition(0);
    }

    public void updateResults(){
        this.results = limeLight.getLatestResult();
    }

    public LLResult getResults(){
        updateResults();
        return results;
    }

    public void setLimelightDriveController(double p, double i, double d, double f) {
        limelightDriveController = new PIDFController(new CustomPIDFCoefficients(p, i, d, f));
    }

    public void setLimelightRotationController(double p, double i, double d, double f) {
        limelightRotationController = new PIDFController(new CustomPIDFCoefficients(p, i, d, f));
    }

    /**
     * make sure that when you call this in the teleOp you arnt trying to also call any sort of gamepad drive
     * this function will move the robot of its own accord
     */

    public void seeknDestroy(@NonNull Follower follower, Telemetry telemetry) {
        updateResults();
        if (results.isValid()) {
            double rotationError = results.getTx();
            double driveError = results.getTy();

            limelightRotationController.updatePosition(rotationError);
            limelightDriveController.updatePosition(driveError);

            double rotationPower = limelightRotationController.runPIDF();
            double drivePower = limelightDriveController.runPIDF();
            telemetry.addLine("Charging");
            telemetry.addData("drive power", drivePower);
            telemetry.addData("Drive error", driveError);
            follower.setTeleOpMovementVectors(drivePower, 0, rotationPower, true);
        } else {
            telemetry.addLine("No valid results!");
        }
        telemetry.update();
    }
}
