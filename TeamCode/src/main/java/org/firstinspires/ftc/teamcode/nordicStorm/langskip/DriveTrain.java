package org.firstinspires.ftc.teamcode.nordicStorm.langskip;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

public class DriveTrain {

    public final Follower follower;

    private final Limelight3A limeLight;

    private LLResult llResult;

    private final PIDFController limelightDriveController;
    private final PIDFController limelightRotationController;

    private boolean isRunningAprilTags;

    private final int blocksPipeLine = 0;
    private final int aprilTagPipeLine = 1;

    // this converts the coordinates returned by the
    // limelight to the coordinate system used by pedro
    private final int xConversion = 0;
    private final int yConversion = 0;

    public Pose position;

    DriveTrain(final HardwareMap hardwareMap) {
        follower = new Follower(hardwareMap);

        limeLight = hardwareMap.get(Limelight3A.class, "lime");

        limeLight.pipelineSwitch(aprilTagPipeLine);

        limeLight.start();

        limelightDriveController = new PIDFController(new CustomPIDFCoefficients(0.03, 0, 0.03, 0.001));
        limelightRotationController = new PIDFController(new CustomPIDFCoefficients(0.03, 0, 0.035, 0.001));

        limelightDriveController.setTargetPosition(-15);
        limelightRotationController.setTargetPosition(0);

        isRunningAprilTags = true;

        position = new Pose();
    }

    public Pose getPosition(final Telemetry telemetry) {
        updatePosition(telemetry);
        return follower.getPose();
    }

    public void seeknDestroy(final @NonNull Follower follower, final Telemetry telemetry) {
        if (isRunningAprilTags) {
            limeLight.pipelineSwitch(blocksPipeLine);
            isRunningAprilTags = false;
        }
        if (llResultsAreGood()) {
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

            follower.setTeleOpMovementVectors(-drivePower, 0, rotationPower, false);
        } else {
            telemetry.addLine("No valid results!");
        }
        telemetry.update();
    }

    public void updatePosition(final Telemetry telemetry) {

        if (!isRunningAprilTags) {
            limeLight.pipelineSwitch(aprilTagPipeLine);
            isRunningAprilTags = true;
        }

        if (llResultsAreGood()) {
            telemetry.addLine("using april tags");

            Pose3D botPose = getLLResult().getBotpose();

            position.setX(metersToInches(botPose.getPosition().x) + xConversion);
            position.setY(metersToInches(botPose.getPosition().y) + yConversion);
            position.setHeading(follower.getTotalHeading());

            follower.setPose(position);
            follower.update();
        } else {
            telemetry.addLine("not using april tags");
            follower.update();
            position = follower.getPose();
        }
    }

    /*
    used internally to make sure "llResults" is not null when called upon
     */
    private LLResult getLLResult() {
        llResult = limeLight.getLatestResult();

        return llResult;
    }

    private boolean llResultsAreGood() {
        return getLLResult() != null && getLLResult().isValid() && getLLResult().getStaleness() < 100;
    }

    private double metersToInches(double meters){
        return meters * 39.3701;
    }

    private double inchesToMeters(double inches){
        return inches * 0.0254;
    }
}
