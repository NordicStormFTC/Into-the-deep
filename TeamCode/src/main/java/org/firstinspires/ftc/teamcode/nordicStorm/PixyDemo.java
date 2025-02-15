package org.firstinspires.ftc.teamcode.nordicStorm;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.nordicStorm.langskip.Langskip;
import org.firstinspires.ftc.teamcode.nordicStorm.pixy.Pixy;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

@TeleOp
public class PixyDemo extends LinearOpMode {

    private Pixy pixy;
    private PIDFController pid;

    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;

    private final double targetX = (double) 315 / 2;

    @Override
    public void runOpMode() throws InterruptedException {
        frontleft = hardwareMap.get(DcMotor.class, "front left");
        frontright = hardwareMap.get(DcMotor.class, "front right");
        backleft = hardwareMap.get(DcMotor.class, "back left");
        backright = hardwareMap.get(DcMotor.class, "back right");

        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);

        pixy = hardwareMap.get(Pixy.class, "pixy");
        pid = new PIDFController(new CustomPIDFCoefficients(0.01, 0, 0, 0.1));
        pid.setTargetPosition(targetX);
        waitForStart();
        while (opModeIsActive()) {
            if(pixy.getBlock().signature == 1){
                pid.updatePosition(pixy.getBlock().centerX);
            } else {
               pid.updatePosition(targetX);
//                telemetry.addData("valid", pixy.hasValidBlock());
//                telemetry.addLine(pixy.getBlock().toString());
            }

            double power = pid.runPIDF();
            
            frontleft.setPower(-power);
            backleft.setPower(-power);
            frontright.setPower(power);
            backright.setPower(power);
            telemetry.addData("power", pid.runPIDF());
            telemetry.addData("error", pid.getError());
            telemetry.update();

        }
    }
}
