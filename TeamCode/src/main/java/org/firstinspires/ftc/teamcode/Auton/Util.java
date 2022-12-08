package org.firstinspires.ftc.teamcode.Auton;

public abstract class Util {
    public static void pause(long time){
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < time + startTime){}
    }
}
