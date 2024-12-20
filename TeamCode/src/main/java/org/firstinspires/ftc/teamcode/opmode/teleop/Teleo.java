package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nordicStorm.langskip.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

@TeleOp
@Config
public class Teleo extends LinearOpMode {

    private DcMotor motor;
    private PIDFController pidf;

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public static double target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "arm");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pidf = new PIDFController(new CustomPIDFCoefficients(p, i, d, f));
        waitForStart();
        while(opModeIsActive()){
            if(target > 10 || target < -10){
                throw new RuntimeException("IF you do that ur arm will literaly implode");
            }
            pidf.setCoefficients(new CustomPIDFCoefficients(p, i, d, f));
            pidf.updatePosition(motor.getCurrentPosition());
            pidf.setTargetPosition(target);
            double power = pidf.runPIDF();
            telemetry.addData("motor",motor.getCurrentPosition());
            motor.setPower(power);
            telemetry.addData("p", pidf.P());
            telemetry.update();
        }
    }
}
