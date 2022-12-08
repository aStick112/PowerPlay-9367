package org.firstinspires.ftc.teamcode.Auton;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.MecanumDrive;

@Autonomous(name="Basic Auto")
public class BasicAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive mecanumDrive = new MecanumDrive(
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"), false);



        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            mecanumDrive.drive(0.70,-0.70,-0.70,0.70);
            pause(600);
            break;
        }
    }

    public void pause(long time){
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < time + startTime){
        }
    }
}
