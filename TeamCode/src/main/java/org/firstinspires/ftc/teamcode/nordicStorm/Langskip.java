package org.firstinspires.ftc.teamcode.nordicStorm;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Langskip {

    public final ArmSubsystem armSubsystem;
    public final VisionSubsystem visionSubsystem;
    public Langskip(HardwareMap hardwareMap, OpType opType){
        visionSubsystem = new VisionSubsystem(hardwareMap, opType);
        armSubsystem = new ArmSubsystem(hardwareMap, opType);
    }
}
