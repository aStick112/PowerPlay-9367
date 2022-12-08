package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MecanumDrive {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    public MecanumDrive(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, boolean b){
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        if(b) {
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void driveVelocity(double powerfL, double powerfR, double powerbL, double powerbR){
        frontLeft.setVelocity(powerfL*1040);
        frontRight.setVelocity(powerfR*1040);
        backLeft.setVelocity(powerbL*1040);
        backRight.setVelocity(powerbR*1040);
    }

    public void drive(double powerfL, double powerfR, double powerbL, double powerbR){
        frontLeft.setPower(powerfL);
        frontRight.setPower(powerfR);
        backLeft.setPower(powerbL);
        backRight.setPower(powerbR);
    }

    public String getVelocity(){
        return " FrontLeft " + frontLeft.getVelocity() +
                "\n FrontRight " + frontRight.getVelocity() +
                "\n BackLeft " + backLeft.getVelocity() +
                "\n backRight " + backRight.getVelocity();
    }

    public void setMode(DcMotor.RunMode runMode){
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }
}
