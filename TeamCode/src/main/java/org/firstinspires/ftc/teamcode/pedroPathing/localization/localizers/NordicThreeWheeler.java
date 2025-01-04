package org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Encoder;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Matrix;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.util.NanoTimer;

public class NordicThreeWheeler extends Localizer {

    private HardwareMap hardwareMap;
    private Pose startPose;
    private Pose displacementPose;
    private Pose currentVelocity;
    private Matrix prevRotationMatrix;
    private NanoTimer timer;
    private long deltaTimeNano;
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder strafeEncoder;
    private Pose leftEncoderPose;
    private Pose rightEncoderPose;
    private Pose strafeEncoderPose;
    private double totalHeading;
    public static double FORWARD_TICKS_TO_INCHES = 0.00052189;//8192 * 1.37795 * 2 * Math.PI * 0.5008239963;
    public static double STRAFE_TICKS_TO_INCHES = 0.00052189;//8192 * 1.37795 * 2 * Math.PI * 0.5018874659;
    public static double TURN_TICKS_TO_RADIANS = 0.00053717;//8192 * 1.37795 * 2 * Math.PI * 0.5;

    public NordicThreeWheeler(HardwareMap map){
        this(map, new Pose());
    }

    public NordicThreeWheeler(HardwareMap map, Pose setStartPose) {
        // TODO: replace these with your encoder positions
        leftEncoderPose = new Pose(3, 0.01, 0);
        rightEncoderPose = new Pose(-2, 0.01, 0);
        strafeEncoderPose = new Pose(0.01, 0.01, Math.toRadians(90));

        hardwareMap = map;

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "par1"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "par2"));
        strafeEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "strafe"));

        leftEncoder.setDirection(Encoder.REVERSE);
        rightEncoder.setDirection(Encoder.REVERSE);
        strafeEncoder.setDirection(Encoder.FORWARD);

        setStartPose(setStartPose);

        timer = new NanoTimer();

        deltaTimeNano = 1;

        displacementPose = new Pose();

        currentVelocity = new Pose();

        totalHeading = 0;

        resetEncoders();
    }

    /**
     * This updates the Encoders.
     */
    public void updateEncoders() {
        leftEncoder.update();
        rightEncoder.update();
        strafeEncoder.update();
    }

    /**
     * This resets the Encoders.
     */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
        strafeEncoder.reset();
    }

    @Override
    public Pose getPose() {
        return MathFunctions.addPoses(startPose, displacementPose);
    }

    // the pedro localizer returns o copy of the velocity which points to a differetn memory allocation, is this the prblem?
    @Override
    public Pose getVelocity() {
        return null;
    }

    @Override
    public Vector getVelocityVector() {
        return null;
    }

    @Override
    public void setStartPose(Pose setStart) {

    }

    @Override
    public void setPose(Pose setPose) {

    }

    @Override
    public void update() {

    }

    @Override
    public double getTotalHeading() {
        return 0;
    }

    @Override
    public double getForwardMultiplier() {
        return 0;
    }

    @Override
    public double getLateralMultiplier() {
        return 0;
    }

    @Override
    public double getTurningMultiplier() {
        return 0;
    }

    @Override
    public void resetIMU() {

    }
}
