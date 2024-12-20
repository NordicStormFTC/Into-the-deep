package org.firstinspires.ftc.teamcode.nordicStorm.langskip;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.nordicStorm.PixyCam;

/**
 * The thought here is that the vision subsystem will produce data for the
 * hardware to react to. Hence the vision subsystem extends the main robot class 'Langskip'
 * and the arm subsystem inherits the data produced by the vision subsystem. The main
 * robot class carries only the subsystems to be used in opmodes, and holds no information
 * besides one object of each subsystem.
 */
public class VisionSubsystem extends Langskip {

    private final Limelight3A limeLight;

    private final PixyCam pixy;

    public LLResult results;

    /**
     * using a package private constructor to
     * maintain immutability.
     * @param hardwareMap publicly instantiate me in Langskip!
     */

    VisionSubsystem(@NonNull HardwareMap hardwareMap) {
        super(hardwareMap);

        pixy = hardwareMap.get(PixyCam.class, "pixy");
        limeLight = hardwareMap.get(Limelight3A.class, "lime");
    }

    public double getX() {
        return results.getBotpose().getPosition().x;
    }

    public double getY() {
        return results.getBotpose().getPosition().y;
    }

}
