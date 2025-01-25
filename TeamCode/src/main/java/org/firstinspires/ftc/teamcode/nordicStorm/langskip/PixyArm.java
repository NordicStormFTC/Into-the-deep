package org.firstinspires.ftc.teamcode.nordicStorm.langskip;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nordicStorm.pixy.Pixy;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

public class PixyArm extends DriveTrain{

    private final DcMotor elbowEncoder;

    private final DcMotor wristEncoder;

    private final DcMotor arm;

    private final double armOffset;

    public final Servo wrist;

    private final Servo elbow;

    private final Servo gripper;

    private final Pixy pixy;

    private boolean elbowIsUp;

    private final PIDFController armPIDF;

     PixyArm(final HardwareMap hardwareMap) {
        super(hardwareMap);

        elbowEncoder = hardwareMap.get(DcMotor.class, "elbow encoder");
        wristEncoder = hardwareMap.get(DcMotor.class, "wrist encoder");

        arm = hardwareMap.get(DcMotor.class, "arm");

        wrist = hardwareMap.get(Servo.class, "wrist");
        elbow = hardwareMap.get(Servo.class, "elbow");
        gripper = hardwareMap.get(Servo.class, "gripper");

        pixy = hardwareMap.get(Pixy.class, "pixy");

        elbowIsUp = true;

        armOffset = arm.getCurrentPosition();

        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        armPIDF = new PIDFController(new CustomPIDFCoefficients(0.0038, 0, 0.0007, 0.00012));

        armPIDF.setTargetPosition(0);
    }

    public synchronized void setScoringPos(final ArmConstants.ScoringPosition scoringPosition) {
        setTarget(105);
        elbow.setPosition(0.5);
        wrist.setPosition(.68);
    }

    public synchronized void putDownArm() {
        elbow.setPosition(ArmConstants.ELBOW_UP);
        setTarget(30);
        setTarget(0);
    }

    public synchronized void putDownElbow() {
        if (elbowIsUp) {
            wrist.setPosition(0.48);
            elbow.setPosition(ArmConstants.ELBOW_DOWN);
            elbowIsUp = false;
        }
    }

    public synchronized void foldInElbow() {
        if (!elbowIsUp) {
            wrist.setPosition(0.5);
            elbow.setPosition(ArmConstants.ELBOW_UP);
            elbowIsUp = true;
        }
    }

    public synchronized void openGripper() {
        gripper.setPosition(ArmConstants.GRIPPER_OPEN);
    }

    public synchronized void closeGripper() {
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
        return arm.getCurrentPosition() - armOffset;
    }

}
