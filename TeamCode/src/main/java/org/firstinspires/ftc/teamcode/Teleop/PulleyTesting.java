package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="Pulley Encoder", group="Testing")
public class PulleyTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Init pulley
        DcMotorEx pulley = hardwareMap.get(DcMotorEx.class, "pulley");
        pulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoder counts kept by motor
        pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motor to run to using encoder
        pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Mode", "Waiting");
        telemetry.update();

        // wait while opmode is active and left motor is busy running to position.

        waitForStart();

        while (opModeIsActive())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder", pulley.getCurrentPosition() + "  busy=" + pulley.isBusy());
            telemetry.update();
            idle();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.



    }
}
