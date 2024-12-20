package org.firstinspires.ftc.teamcode.nordicStorm.langskip;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

/**
 * The goal here is to have only one robot info class instantiated
 * in our OpModes. Langskip means 'boat' in old norse, and publicly
 * contains all of our subsystems. Users should not instantiate
 * individual subsystems in OpModes.
 */
public class Langskip {

    public final ArmSubsystem armSubsystem;

    public final VisionSubsystem visionSubsystem;

    public final Follower follower;

    public Langskip(HardwareMap hardwareMap) {

        visionSubsystem = new VisionSubsystem(hardwareMap);

        armSubsystem = new ArmSubsystem(hardwareMap);

        follower = new Follower(hardwareMap);
    }
}
