package org.firstinspires.ftc.teamcode.nordicStorm.langskip;

public final class ArmConstants {

    public static final double GOBUILDA_ENCODER_RESOLUTION = 751.8;
    public static final double GOBUILDA_INTERNAL_GEAR_RATIO = (double) 9 / 26;
    public static final double ARM_BELTING_RATIO = 1.4;

    public static final double TICKS_PER_REVOLUTION = (GOBUILDA_ENCODER_RESOLUTION * GOBUILDA_INTERNAL_GEAR_RATIO) * ARM_BELTING_RATIO * 3;//why 3?? I dont understand!! qwjnrfpiqenrglxkjqwnrxg;ljqwn rgl;knqoirgi

    public static final double TICKS_TO_DEGREES = (360 / TICKS_PER_REVOLUTION);
    public static final double DEGREES_TO_TICKS = (TICKS_PER_REVOLUTION / 360);

    public static final double MAX_ARM_ANGLE = 180;
    public static final double MIN_ARN_ANGLE = 0;

    public static final int elbowTolerance = 75;
    public static final int wristTolerance = 50;

    public static final int elbowUp = -1;
    public static final int elbowDown = 1;

    public static final int wristUp = 1;
    public static final int  wristDown = -1;

    //---------- elbow encoder positions corresponding to the sheet
    public static final int elbowID1 = 3580;
    public static final int elbowID2 = 5820;
    public static final int elbowID3 = 1300;
    public static final int elbowID4 = 3026;
    public static final int elbowID5 = 1850;

    public static final int wristID1 = 3030;
    public static final int wristID2 = -130;
    public static final int wristID3 = 2000;
    public static final int wristID4 = 3550;
    public static final int wristID5 = 3200;
}

