package org.firstinspires.ftc.teamcode.opmode.example;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nordicStorm.langskip.Langskip;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

/**
 * This is an example teleop that showcases movement and control of two servos and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */
@Config
@TeleOp
public class ExampleTeleop_RobotCentric extends OpMode {
    private Langskip langskip;
    private final Pose startPose = new Pose(0,0,0);

    public static double dp =0;
    public static double di =0;
    public static double dd =0;
    public static double df=0;
    public static double rp=0;
    public static double ri=0;
    public static double rd=0;
    public static double rf=0;

    /** This method is call once when init is played, it initializes the follower and subsystems **/
    @Override
    public void init() {
      langskip = new Langskip(hardwareMap);
      langskip.follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        langskip.follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
//        langskip.visionSubsystem.setLimelightDriveController(dp,di,dd,df);
//        langskip.visionSubsystem.setLimelightRotationController(rp,ri,rd,rf);

//        if(langskip.visionSubsystem.getResults().isValid()){
//            telemetry.addData("tx", langskip.visionSubsystem.getResults().getTx());
//        }
//        telemetry.addData("VAKLID", langskip.visionSubsystem.getResults().isValid());

        if(gamepad1.a){
            langskip.visionSubsystem.seeknDestroy(langskip.follower,telemetry);
        } else if(
                !gamepad1.a){
            langskip.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, 0, true);
        }
        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */


        langskip.follower.update();

        /* Open claw on Left Bumper Press */
        if (gamepad1.left_bumper) {
        }

        /* Close claw on Right Bumper Press */
        if (gamepad1.right_bumper) {
        }

        /* Ground Claw Pivot Position on A Press */
        if (gamepad1.a) {
        }

        /* Scoring Claw Pivot Position on B Press */
        if (gamepad1.b) {
        }

        /* This could be paired with a PIDF to set the target position of the lift in teleop.
         * For this, you would have to update the lift pid and make sure to initializes the lift subsystem.
         */

        /*
        if (gamepad1.left_trigger > 0.5) {
            lift.setTarget(lTarget-50);
        }

        if (gamepad1.right_trigger > 0.5) {
            lift.setTarget(lTarget+50);
        }
        */

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", langskip.follower.getPose().getX());
        telemetry.addData("Y", langskip.follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(langskip.follower.getPose().getHeading()));

        /* Telemetry Outputs of our ClawSubsystem */


        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}
