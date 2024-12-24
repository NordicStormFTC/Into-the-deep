package org.firstinspires.ftc.teamcode.nordicStorm.langskip;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

/**
 * The thought here is that the vision subsystem will produce data for the
 * hardware to react to. Hence the vision subsystem extends the main robot class 'Langskip'
 * and the arm subsystem inherits the data produced by the vision subsystem. The main
 * robot class carries only the subsystems to be used in opmodes, and holds no information
 * besides one object of each subsystem.
 */
public class ArmSubsystem extends VisionSubsystem {

    private final DcMotor arm;

    public final Servo wrist;

    public final Servo elbow;

    public double target = 0; // the arm can only go in range [10,10]

    /**
     * we use one class private PIDF to be used by the arm subsystem.
     * again you could instantiate a new PIDF controller every OpMode
     * you run your arm in, but this seems cleaner.
     */
    private final PIDFController armPIDF;

    /**
     * using a package private constructor to
     * maintain immutability. Publicly instantiate me as final in Langskip!
     *
     * @param hardwareMap hardware map for the arm and servos.
     */

    ArmSubsystem(HardwareMap hardwareMap) {
        super(hardwareMap);

        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        elbow = hardwareMap.get(Servo.class, "elbow");

        armPIDF = new PIDFController(new CustomPIDFCoefficients(0, 0, 0, 0));
    }

    /**
     * call this any time to set the arms target position
     *
     * @param target target position
     */
    public void setTarget(double target) throws RuntimeException {
        this.target = target;

        if (target > RobotConstants.maxArmPose || target < RobotConstants.minArmPose) {
            throw new RuntimeException("NO! BAD! The arm can only go from [10, 10]");
        }
    }

    /**
     * this is the function to be called in loop
     * it will set power once every time you call it,
     * so again, it must be put in the loop of the OpMode
     */
    public void runArm() {
        armPIDF.updatePosition(arm.getCurrentPosition());
        armPIDF.setTargetPosition(target);
        double power = armPIDF.runPIDF();
        arm.setPower(power);
    }


    /**
     * you could do this in the OpMode, however it seems
     * cleaner to do it here. this sets the PIDF coefficiants
     */
    public void setArmPIDF(double p, double i, double d, double f) {
        armPIDF.setCoefficients(new CustomPIDFCoefficients(p, i, d, f));
        double[] pidCoefficients = {p, i, d, f};

        /*
         * checks for any silly mistakes
         */
        for (double term : pidCoefficients) {
            if (term < 0) {
                throw new RuntimeException("NO! BAD! PID Coefficients cannot be negative");
            }
        }
        if (p > 0.1) {
            throw new RuntimeException("NO! BAD! The arm P term should not be higher than 0.1!!");
        }
        if (i > 0) {
            throw new RuntimeException("NO! DO NOT USE AN I TERM FOR TUNING THE ARM!");
        }
    }

    public double getArmPosition() {
        return arm.getCurrentPosition();
    }

    public void setWrist(double target){

    }


    //-------------------------------- This deals with the first segment of our arm

    private double calculateArmAngle() {
        return 300 * arm.getCurrentPosition();
    }

    private double calculateElbowJointXPos() {
        double armAngle = calculateArmAngle();

        return Math.sin(armAngle) * RobotConstants.length1;
    }

    private double calculateElbowJointYPos() {
        double armAngle = calculateArmAngle();

        return Math.tan(armAngle) * RobotConstants.length1;
    }

    //--------------------------------

    //--------------------------------this deals with the second segment of our arm

    private double calculateElbowAngle() {
        return 300 * elbow.getPosition();
    }

    private double calculateWristJointXPos() {
        double elbowAngle = calculateElbowAngle();

        return Math.sin(elbowAngle) * RobotConstants.length2 + calculateElbowJointXPos();
    }

    private double calculateWristJointYPos() {
        double elbowAngle = calculateElbowAngle();

        return (Math.tan(elbowAngle) * RobotConstants.length2) + calculateElbowJointYPos();
    }

    //--------------------------------

    //--------------------------------this deals with our wrist

    public double calculateWristAngle() {
        return 200 * wrist.getPosition();
    }

    public double calculateGripperX() {
        double wristAngle = calculateWristAngle();

        return (Math.sin(wristAngle) * RobotConstants.length3) + calculateWristJointXPos();
    }

    public double calculateGripperY() {
        double wristAngle = calculateWristAngle();

        return (Math.tan(wristAngle) * RobotConstants.length3) + calculateWristJointYPos();
    }
    //-------------------------------

    //-------------------------------deals with the optimzation of the joint angles

}
