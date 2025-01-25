package org.firstinspires.ftc.teamcode.nordicStorm.langskip;

public final class ArmConstants {

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
    public final double FOLDED_WRIST = 0.5;
    public final double GRABBING_WRIST = 0.47;
    public final double LOW_BASKET_WRIST = 0.69;

    public enum ScoringPosition {
        CLIP,
        LOW_BASKET
    }
}

