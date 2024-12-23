package org.firstinspires.ftc.teamcode.nordicStorm.langskip;

/**
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 9/8/2024
 */

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
@Config // this allows us to directly interface with variables using ftc dashboard
/** Everything that we want to store globally, for example positions of servos, motors, etc. goes in here. **/
public class RobotConstants {

public static final Pose bar = new Pose(10,10);

public static final int maxArmPose = 10;
public static final int minArmPose = -10;

    static final double length1 = 10;
    static final double length2 = 10;
    static final double length3 = 10;

    public static final double restingWrist = 0.71;
    public static double restingElbow = 0.71;

}
