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

    private PIDFController limelightDriveController;
    private PIDFController limelightRotationController;

    private PIDFController pixyDriveController;
    private PIDFController pixyRotationController;

    /**
     * using a package private constructor to
     * maintain immutability.
     *
     * @param hardwareMap publicly instantiate me in Langskip!
     */

    VisionSubsystem(@NonNull HardwareMap hardwareMap) {
        limeLight = hardwareMap.get(Limelight3A.class, "lime");

       // pixy = hardwareMap.get(PixyCam.class, "pixy");

        limeLight.pipelineSwitch(0);

        limeLight.start();

        limelightDriveController = new PIDFController(new CustomPIDFCoefficients(0.03, 0, 0.03, 0.001));
        limelightRotationController = new PIDFController(new CustomPIDFCoefficients(0, 0, 0, 0));

        pixyDriveController = new PIDFController(new CustomPIDFCoefficients(0,0,0,0));
        pixyRotationController = new PIDFController(new CustomPIDFCoefficients(0,0,0,0));

        limelightDriveController.setTargetPosition(0);
        limelightRotationController.setTargetPosition(0);
    }

    /**
     * call this method in the loop of your opMode!
     */
    public void updateCameras(){
        llResult = limeLight.getLatestResult();
        //pixyData = pixy.read();
    }

    public LLResult getLLResult(){
        updateCameras();
        return llResult;
    }

    public List<PixyBlock> getPixyResults(){
        updateCameras();
        return pixyData;
    }

    /**
     * these next four methods are used to tune the alignment controls of the cameras.
     * once a user has found the correct coefficients, they should mark the controllers
     * as final, and instantiate them in the constructor, with their experimentally found coefficients.
     */
    public void setLimelightDriveController(double p, double i, double d, double f) {
        limelightDriveController = new PIDFController(new CustomPIDFCoefficients(p, i, d, f));
    }

    public void setLimelightRotationController(double p, double i, double d, double f) {
        limelightRotationController = new PIDFController(new CustomPIDFCoefficients(p, i, d, f));
    }

    public void setPixyDriveController(double p, double i, double d, double f){
        pixyDriveController = new PIDFController(new CustomPIDFCoefficients(p,i,d,f));
    }

    public void setPixyRotationController(double p, double i, double d, double f){
        pixyRotationController = new PIDFController(new CustomPIDFCoefficients(p,i,d,f));
    }

    /**
     * make sure that when you call this in the teleOp you arnt trying to also call any sort of gamepad drive
     * this function will move the robot of its own accord
     */

    public void seeknDestroy(@NonNull Follower follower, Telemetry telemetry) {
        updateCameras();
        if (llResult.isValid()) {
            double rotationError = llResult.getTx();
            double driveError = llResult.getTy();

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
