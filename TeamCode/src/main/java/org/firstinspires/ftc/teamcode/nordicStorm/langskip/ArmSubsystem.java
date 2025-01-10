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

    private final double offset;

    public final Servo wrist;

    private final Servo elbow;

    private final Servo gripper;

    private boolean elbowIsUp;

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
    ArmSubsystem(final HardwareMap hardwareMap) {
        super(hardwareMap);

        arm = hardwareMap.get(DcMotor.class, "arm");

        wrist = hardwareMap.get(Servo.class, "wrist");
        elbow = hardwareMap.get(Servo.class, "elbow");
        gripper = hardwareMap.get(Servo.class, "gripper");

        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        armPIDF = new PIDFController(new CustomPIDFCoefficients(0.0038, 0, 0.00095, 0.00012));

        offset = arm.getCurrentPosition();

        armPIDF.setTargetPosition(0);

        elbowIsUp = true;
    }

    public void setScoringPos(ScoringPosition scoringPosition) {
        if (scoringPosition == ScoringPosition.LOW_BASKET) {
            foldInElbow();
            setTarget(95);
            wrist.setPosition(0.055);
        }
    }

    public void putDownElbow() {
        if (elbowIsUp) {
            wrist.setPosition(ArmConstants.WRIST_STRAIGHT + 0.01);
            elbow.setPosition(ArmConstants.ELBOW_DOWN);
            elbowIsUp = false;
        }
    }

    public void foldInElbow() {
        if (!elbowIsUp) {
            wrist.setPosition(ArmConstants.WRIST_DOWN);
            elbow.setPosition(ArmConstants.ELBOW_UP);
            elbowIsUp = true;
        }
    }

    public void grabPiece() {
        putDownElbow();
        openGripper();
        wrist.setPosition(ArmConstants.WRIST_GRAB);
        closeGripper();
    }

    public void openGripper() {
        gripper.setPosition(ArmConstants.GRIPPER_OPEN);
    }

    public void closeGripper() {
        gripper.setPosition(ArmConstants.GRIPPER_CLOSE);
    }

    /**
     * call this any time to set the arms target position
     *
     * @param degrees target position in degrees, 30 is resting down
     */
    public void setTarget(final double degrees) throws RuntimeException {
        // see the arm constants section for a valid target range.
        final double targetTicks = degrees * ArmConstants.DEGREES_TO_TICKS;

        if ((degrees) > ArmConstants.MAX_ARM_ANGLE || (degrees) < ArmConstants.MIN_ARN_ANGLE) {
            throw new RuntimeException("NO! BAD! The arm can only go from [" + ArmConstants.MIN_ARN_ANGLE + "," + ArmConstants.MAX_ARM_ANGLE + "]");
        } else {
            armPIDF.setTargetPosition(targetTicks);
        }
    }

    /**
     * this is the function to be called in loop
     * it will set power once every time you call it,
     * so again, it must be put in the loop of the OpMode
     */
    public void runArm(Telemetry telemetry) {
        armPIDF.updatePosition(getArmPositionTicks());
        double power = armPIDF.runPIDF();
        telemetry.addData("posiiton", getArmPositionTicks());
        telemetry.addData("Power", power);
        telemetry.addData("error", armPIDF.getError());
        telemetry.addData("PID Target", armPIDF.getTargetPosition() * ArmConstants.TICKS_TO_DEGREES);
        arm.setPower(power);
    }


    /**
     * you could do this in the OpMode, however it seems
     * cleaner to do it here. this sets the PIDF coefficients for the arm.
     * this method can be used for the initial testing to find the correct coefficients.
     * after this, the coefficients can be declared in the constructor, and marked as final,
     * at which point you never have to call this method again.
     */
    public void setArmPIDF(final double p, final double i, final double d, final double f) throws RuntimeException {
        double[] pidCoefficients = {p, i, d, f};
        /*
         * checks for any silly tuning mistakes
         */
        for (double term : pidCoefficients) {
            if (term < 0) {
                throw new RuntimeException("NO! BAD! PID Coefficients cannot be negative");
            }
        }
        if (p > 0.01) {
            throw new RuntimeException("NO! BAD! The arm P term should not be higher than 0.1!!");
        }
        if (i != 0) {
            throw new RuntimeException("NO! DO NOT USE AN I TERM FOR TUNING THE ARM!");
        }
        armPIDF.setCoefficients(new CustomPIDFCoefficients(p, i, d, f));
    }

    private double getArmPositionTicks() {
        return arm.getCurrentPosition() - offset;
    }

    public static final class ArmConstants {

        public static final double GOBUILDA_ENCODER_RESOLUTION = 751.8;
        public static final double GOBUILDA_INTERNAL_GEAR_RATIO = (double) 9 / 26;
        public static final double ARM_BELTING_RATIO = 3;

        public static final double TICKS_PER_REVOLUTION = (GOBUILDA_ENCODER_RESOLUTION * GOBUILDA_INTERNAL_GEAR_RATIO) * ARM_BELTING_RATIO * 3;//why 3?? I dont understand! qwjnrfpiqenrglxkjqwnrxg;ljqwn rgl;knqoirgi

        public static final double TICKS_TO_DEGREES = (360 / TICKS_PER_REVOLUTION);
        public static final double DEGREES_TO_TICKS = (TICKS_PER_REVOLUTION / 360);

        public static final double MAX_ARM_ANGLE = 180;
        public static final double MIN_ARN_ANGLE = 0;

        public static final double ELBOW_UP = 0;
        public static final double ELBOW_DOWN = 1;

        public static final double GRIPPER_CLOSE = 1;
        public static final double GRIPPER_OPEN = 0;

        //---------- Every 0.01 is like a sixth of a rotation on the wrist
        public static final double WRIST_DOWN = 0; // this is also the position used when folding in the elbow
        public static final double WRIST_STRAIGHT = 0.03;
        public static final double WRIST_GRAB = 0.02;
    }

    public enum ScoringPosition {
        CLIP,
        LOW_BASKET
    }
}
