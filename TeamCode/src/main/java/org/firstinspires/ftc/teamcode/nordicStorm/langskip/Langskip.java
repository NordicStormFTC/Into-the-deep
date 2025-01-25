package org.firstinspires.ftc.teamcode.nordicStorm.langskip;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

/**
 * The goal here is to have only one robot info class instantiated
 * in our OpModes. Langskip means 'boat' in old norse, and publicly
 * contains all of our subsystems. Users should not instantiate
 * individual subsystems in OpModes and access them through langskip.
 */
public class Langskip {

//    public final VisionSubsystem visionSubsystem;
//
//    public final ArmSubsystem armSubsystem;
//
//    public final Follower follower;
//
//    public final Pose startPose = new Pose(0,0,0);


    public final DriveTrain driveTrain;

    /**
     *
     * @param hardwareMap the hardware map for our subsystems to use. This provides the same instance of the hardware map to all subsystems
     */
    public Langskip(@NonNull final HardwareMap hardwareMap) {
//        follower = new Follower(hardwareMap);
//
//        visionSubsystem = new VisionSubsystem(hardwareMap);
//
//        armSubsystem = new ArmSubsystem(hardwareMap);

        driveTrain = new DriveTrain(hardwareMap);
    }
}
