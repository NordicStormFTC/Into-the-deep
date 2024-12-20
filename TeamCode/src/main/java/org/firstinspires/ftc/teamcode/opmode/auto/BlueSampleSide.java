package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.nordicStorm.AutonomousHelpers.buildLine;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.nordicStorm.HeadingInterpolation;
import org.firstinspires.ftc.teamcode.nordicStorm.langskip.Langskip;
import org.firstinspires.ftc.teamcode.nordicStorm.langskip.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class BlueSampleSide extends OpMode {

    private Langskip robot;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /**
     * This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method.
     */
    private int pathState;

    /**
     * used for the arm subsystem
     */
    private PIDFController pidfController;

    private double targetArmPosition = 0;

    /**
     * Start Pose of our robot
     */
    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));

    /**
     * Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve.
     */
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));

    private final Path[] paths = new Path[10];

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path path;
    private PathChain pathChain;

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pidfController = new PIDFController(new CustomPIDFCoefficients(0, 0, 0, 0));
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        robot = new Langskip(hardwareMap);
        robot.follower.setStartingPose(startPose);

        buildPaths();
    }

    /**
     * Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts.
     **/
    public void buildPaths() {
        paths[0] = buildLine(
                RobotConstants.bar,
                new Pose(41.063, 65.625, 180),
                HeadingInterpolation.CONSTANT
        );

    }

    /**
     * This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setPathState(1);
                break;
            case 1:

                if (true) {
                    //do thing
                    robot.follower.setPose(new Pose(robot.visionSubsystem.getX(), robot.visionSubsystem.getY()));
                    setPathState(2);
                }
                break;
            case 2:
                setPathState(3);
                break;
            case 3:
                setPathState(4);
                break;
            case 4:
                setPathState(5);
                break;
            case 5:
                setPathState(6);
                break;
            case 6:
                setPathState(7);
                break;
            case 7:
                setPathState(8);
                break;
            case 8:
                setPathState(9);
                break;
        }
    }

    /**
     * These change the states of the paths and actions
     * It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        pidfController.updatePosition(robot.armSubsystem.getArmPosition());
        pidfController.setTargetPosition(targetArmPosition);
        double power = pidfController.runPIDF();
        robot.armSubsystem.setArmPower(power);
        // These loop the movements of the robot
        robot.follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", robot.follower.getPose().getX());
        telemetry.addData("y", robot.follower.getPose().getY());
        telemetry.addData("heading", robot.follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}
