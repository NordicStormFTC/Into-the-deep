package org.firstinspires.ftc.teamcode.nordicStorm.langskip;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    public double offset = 0;

    public final Servo wrist;

    public final Servo elbow;

    public double targetTicks = 0; // see the arm constants section for a valid target range.

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
     * @param hardwareMap hardware map for the arms motor and servos.
     */

    ArmSubsystem(HardwareMap hardwareMap) {
        super(hardwareMap);

        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        elbow = hardwareMap.get(Servo.class, "elbow");

        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        armPIDF = new PIDFController(new CustomPIDFCoefficients(0, 0, 0, 0));

        setArmPIDF(0.006, 0, 0.0009, 0.0001);
    }

    /**
     * call this any time to set the arms target position
     *
     * @param degrees target position in degrees, 30 is resting down
     */
    public void setTargetTicks(double degrees) throws RuntimeException {
        armPIDF.reset();
        degrees -= 30;
        targetTicks = degrees * ArmConstants.DEGREES_TO_TICKS;

        if (degrees > ArmConstants.MAX_ARM_ANGLE || degrees < ArmConstants.MIN_ARN_ANGLE) {
            throw new RuntimeException("NO! BAD! The arm can only go from [" + ArmConstants.MIN_ARN_ANGLE + "," + ArmConstants.MAX_ARM_ANGLE + "]");
        } else {
            armPIDF.setTargetPosition(targetTicks);
        }
    }

    public double getArmAngle() {
        return calculateArmAngle();
    }

    /**
     * this is the function to be called in loop
     * it will set power once every time you call it,
     * so again, it must be put in the loop of the OpMode
     */
    public void runArm(Telemetry telemetry) {
        armPIDF.updatePosition(getArmPositionTicks());
        double power = armPIDF.runPIDF();
        telemetry.addData("Power", power);
        telemetry.addData("error", armPIDF.getError());
        telemetry.addData("PID Target", armPIDF.getTargetPosition());
        arm.setPower(power);
    }


    /**
     * you could do this in the OpMode, however it seems
     * cleaner to do it here. this sets the PIDF coefficients for the arm.
     * this method can be used for the initial testing to find the correct coefficients.
     * after this, the coefficients can be declared in the constructor, and marked as final,
     * at which point you never have to call this method again.
     */
    public void setArmPIDF(double p, double i, double d, double f) throws RuntimeException {
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
        if (i != 0) {
            throw new RuntimeException("NO! DO NOT USE AN I TERM FOR TUNING THE ARM!");
        }
        armPIDF.setCoefficients(new CustomPIDFCoefficients(p, i, d, f));
    }

    public double getArmPositionTicks() {
        return arm.getCurrentPosition() - offset;
    }

    public double getAbsoluteArmTicks() {
        return arm.getCurrentPosition();
    }

    public void setWrist(double target) {

    }

    //-------------------------------- This deals with the first segment of our arm

    private double calculateArmAngle() {
        return (getArmPositionTicks() * ArmConstants.TICKS_TO_DEGREES) + ArmConstants.RESTING_ARM_ANGLE;
    }

    private double calculateElbowJointXPos() {
        double armAngle = calculateArmAngle();

        return Math.sin(armAngle) * ArmConstants.length1;
    }

    private double calculateElbowJointYPos() {
        double armAngle = calculateArmAngle();

        return Math.tan(armAngle) * ArmConstants.length1;
    }

    //--------------------------------

    //--------------------------------this deals with the second segment of our arm

    private double calculateElbowAngle() {
        // return 300 * elbow.getPosition();
        return 2;
    }

    private double calculateWristJointXPos() {
        double elbowAngle = calculateElbowAngle();

        return Math.sin(elbowAngle) * ArmConstants.length2 + calculateElbowJointXPos();
    }

    private double calculateWristJointYPos() {
        double elbowAngle = calculateElbowAngle();

        return (Math.tan(elbowAngle) * ArmConstants.length2) + calculateElbowJointYPos();
    }

    //--------------------------------

    //--------------------------------this deals with our wrist

    public double calculateWristAngle() {
        //return 200 * wrist.getPosition();
        return 1;
    }

    public double calculateGripperX() {
        double wristAngle = calculateWristAngle();

        return (Math.sin(wristAngle) * ArmConstants.length3) + calculateWristJointXPos();
    }

    public double calculateGripperY() {
        double wristAngle = calculateWristAngle();

        return (Math.tan(wristAngle) * ArmConstants.length3) + calculateWristJointYPos();
    }
    //-------------------------------

    //-------------------------------deals with the optimzation of the joint angles

    public static class ArmConstants {
        public static final double GOBUILDA_ENCODER_RESOLUTION = 751.8;
        public static final double GOBUILDA_INTERNAL_GEAR_RATIO = (double) 9 / 26;
        public static final double ARM_BELTING_RATIO = 3;

        public static final int RESTING_ARM_ANGLE = 30; // in degrees

        public static final double TICKS_PER_REVOLUTION = (GOBUILDA_ENCODER_RESOLUTION * GOBUILDA_INTERNAL_GEAR_RATIO) * ARM_BELTING_RATIO * 3;//why 3?? I dont understand qwjnrfpiqenrglxkjqwnrxg;ljqwn rgl;knqoirgi
        //(751.8 * 9/26) * 3/3;

        public static final double TICKS_TO_DEGREES = (360 / TICKS_PER_REVOLUTION);
        public static final double DEGREES_TO_TICKS = (TICKS_PER_REVOLUTION / 360);

        public static final double MAX_ARM_ANGLE = 180;
        public static final double MIN_ARN_ANGLE = 30;

        static final double length1 = 10;
        static final double length2 = 10;
        static final double length3 = 10;

        public static final double restingWrist = 0.71;
        public static double restingElbow = 0.71;
    }
}
