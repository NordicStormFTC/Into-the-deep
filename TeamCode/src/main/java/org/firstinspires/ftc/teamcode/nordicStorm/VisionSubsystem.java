package org.firstinspires.ftc.teamcode.nordicStorm;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

/**
 The thought here is that the vision subsystem will produce data for the
 hardware to react to. Hence the vision subsystem extends the main robot class 'Langskip'
 and the arm subsystem inherits the data produced by the vision subsystem. The main
 robot class carries only the subsystems to be used in opmodes, and holds no information
 besides one object of each subsystem.
 */
public class VisionSubsystem extends Langskip {

    private final OpType opType;
    private final Limelight3A limeLight;

    public LLResult results;

    public VisionSubsystem(@NonNull HardwareMap hardwareMap, OpType opType){
        super(hardwareMap, opType);
        this.opType = opType;

        limeLight = hardwareMap.get(Limelight3A.class, "lime");
    }





}
