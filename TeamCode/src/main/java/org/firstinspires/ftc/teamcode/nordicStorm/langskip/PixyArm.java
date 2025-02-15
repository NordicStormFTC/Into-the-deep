package org.firstinspires.ftc.teamcode.nordicStorm.langskip;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nordicStorm.pixy.Pixy;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

public class PixyArm extends DriveTrain {

    private final DcMotor arm;

    private final double armOffset;

    public final CRServo wrist;

    public final CRServo elbow;

    private final Servo gripper;

    private final Pixy pixy;

    private final PIDFController armPIDF;
    private final PIDFController elbowPIDF;
    private final PIDFController wristPIDF;

    private final int elbowOctoQuadID = 1;

    private final int wristOctoQuadID = 0;

    PixyArm(final HardwareMap hardwareMap) {
        super(hardwareMap);

        arm = hardwareMap.get(DcMotor.class, "arm");

        wrist = hardwareMap.get(CRServo.class, "wrist");
        elbow = hardwareMap.get(CRServo.class, "elbow");
        gripper = hardwareMap.get(Servo.class, "gripper");

        pixy = hardwareMap.get(Pixy.class, "pixy");

        armOffset = arm.getCurrentPosition();

        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        armPIDF = new PIDFController(new CustomPIDFCoefficients(0.003, 0, 0.0007, 0.00012));
        elbowPIDF = new PIDFController(new CustomPIDFCoefficients(0.00095, 0, 0.0001, 0.00055));
        wristPIDF = new PIDFController(new CustomPIDFCoefficients(0.00035, 0, 0.0000001, 0.0025));

        armPIDF.setTargetPosition(0);
        elbow.setPower(0);
        wrist.setPower(0);
    }

    public boolean hasBlock() {
        return pixy.getBlock().isValid();
    }

    public void turnOnLamps() {
        pixy.turnOnLamps();
    }

    public void turnOffLamps() {
        pixy.turnOffLamps();
    }

    public void pixySeek(Telemetry t) {
        if (pixy.getBlock().isValid()) {
            double yError = pixy.getBlock().centerY;
            double xError = pixy.getBlock().centerX;
            t.addData("yError", yError);
            t.addData("xError", xError);
        }
    }

    public void runArm(Telemetry t){
        runElbow(t);
        runShoulder(t);
        runWrist(t);
    }

    /**
     * here me out, this is the very expanded easy to read function that moves our arm to a given position.
     * on our little sheet in the lab, we have a diagram of arm positions "poseIDs" each case will set the continuous rotation servos
     * to forwards, backwards or off depending on the joint encoders reading.
     *
     * @param poseID the position on the diagram to set the arm to
     */
    public void setArm(final int poseID) {
        if (poseID == 1) {
            setShoulderTarget(0);
            setElbowTarget(ArmConstants.elbowID1);
            setWristTarget(ArmConstants.wristID1);
        }
        if (poseID == 2) {
            setShoulderTarget(0);
            setElbowTarget(ArmConstants.elbowID2);
            setWristTarget(ArmConstants.wristID2);
        }
        if (poseID == 3) {
            setShoulderTarget(90);
            setElbowTarget(ArmConstants.elbowID3);
            setWristTarget(ArmConstants.wristID3);
        }
        if (poseID == 4) {
            setElbowTarget(0);
            setElbowTarget(ArmConstants.elbowID4);
            setWristTarget(ArmConstants.wristID4);
        }
        if(poseID == 5){
            setElbowTarget(ArmConstants.elbowID5);
            setShoulderTarget(130);
            setWristTarget(ArmConstants.wristID5);
        }
    }

    public synchronized void openGripper() {
        //gripper.setPosition(ArmConstants.GRIPPER_OPEN);
    }

    public synchronized void closeGripper() {
        //gripper.setPosition(ArmConstants.GRIPPER_CLOSE);
    }

    /**
     * call this any time to set the shoulders target position
     *
     * @param degrees target position in degrees, 30 is resting down
     */
    public void setShoulderTarget(final double degrees) throws RuntimeException {
        // see the arm constants section for a valid target range.
        final double targetTicks = degrees * ArmConstants.DEGREES_TO_TICKS;

        if ((degrees) > ArmConstants.MAX_ARM_ANGLE || (degrees) < ArmConstants.MIN_ARN_ANGLE) {
            throw new RuntimeException("NO! BAD! The arm can only go from [" + ArmConstants.MIN_ARN_ANGLE + "," + ArmConstants.MAX_ARM_ANGLE + "]");
        } else {
            armPIDF.setTargetPosition(targetTicks);
        }
    }

    /**
     * this is used to run a servo... we get position data from a bore encoder.
     *
     * @param target the target position for the elbow
     */
    public void setElbowTarget(final double target) {
        elbowPIDF.setTargetPosition(target);
    }

    /**
     * this is used to run a servo... we get position data from a bore encoder.
     *
     * @param target the target position for the wrist
     */
    public void setWristTarget(final double target) {
        wristPIDF.setTargetPosition(target);
    }

    /**
     * this is the function to be called in loop
     * it will set power once every time you call it,
     * so again, it must be put in the loop of the OpMode
     */
    public void runShoulder(Telemetry telemetry) {
        armPIDF.updatePosition(getArmPositionTicks());
        double power = armPIDF.runPIDF();
        telemetry.addData("Shoulder Position: ", getArmPositionTicks());
        telemetry.addData("Shoulder Power: ", power);
        telemetry.addData("Shoulder Error: ", armPIDF.getError());
        telemetry.addData("Shoulder PID Target: ", armPIDF.getTargetPosition() * ArmConstants.TICKS_TO_DEGREES);
        arm.setPower(power);

    }

    public void runElbow(Telemetry telemetry) {
        elbowPIDF.updatePosition(getElbowPosition());
        double power = elbowPIDF.runPIDF();
        telemetry.addData("position", getElbowPosition());
        telemetry.addData("Power", power);
        telemetry.addData("error", elbowPIDF.getError());
        telemetry.addData("PID Target", elbowPIDF.getTargetPosition());
        elbow.setPower(-power);
    }

    public void runWrist(Telemetry telemetry) {
        wristPIDF.updatePosition(getWristPosition());
        double power = wristPIDF.runPIDF();
        telemetry.addData("wrist Position: ", getWristPosition());
        telemetry.addData("wrist Power: ", power);
        telemetry.addData("wrist Error: ", wristPIDF.getError());
        telemetry.addData("wrist PID Target: ", wristPIDF.getTargetPosition());
        wrist.setPower(power);
    }

    /**
     * you could do this in the OpMode, however it seems
     * cleaner to do it here. this sets the PIDF coefficients for the arm.
     * this method can be used for the initial testing to find the correct coefficients.
     * after this, the coefficients can be declared in the constructor, and marked as final,
     * at which point you never have to call this method again.
     */
    private void setArmPIDF(final double p, final double i, final double d, final double f) throws
            RuntimeException {
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

    public double getElbowPosition() {
        return octoquad.readSinglePosition(elbowOctoQuadID);
    }

    public double getWristPosition() {
        return -1 * octoquad.readSinglePosition(wristOctoQuadID);
    }

    public double getArmPositionTicks() {
        return arm.getCurrentPosition() - armOffset;
    }
}
