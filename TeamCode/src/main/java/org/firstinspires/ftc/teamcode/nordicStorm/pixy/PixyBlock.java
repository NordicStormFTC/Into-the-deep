package org.firstinspires.ftc.teamcode.nordicStorm.pixy;

import androidx.annotation.NonNull;

public class PixyBlock {

    public int sync;
    public int checksum;
    public int signature;
    public int centerX;
    public int centerY;
    public int width;
    public int height;
    public int angle;
    public int trackingIndex;
    public int age;

    public boolean isValid(){
        return true;
    }

    @NonNull
    public String toString(){
        return "[ signature: " + signature + "\ncenterX: " + centerX + "\ncenterY: " + centerY + "\nwidth: " + width + "\nheight: " + height + "\nage: " + age + " ]";
    }
}