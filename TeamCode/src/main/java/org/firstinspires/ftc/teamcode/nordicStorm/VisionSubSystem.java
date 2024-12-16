package org.firstinspires.ftc.teamcode.nordicStorm;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VisionSubsystem {

    private final OpType opType;

    private final Limelight3A limeLight;

    public LLResult results;

    public VisionSubsystem(@NonNull HardwareMap hardwareMap, OpType opType){
        this.opType = opType;

        limeLight = hardwareMap.get(Limelight3A.class, "lime");
    }





}
