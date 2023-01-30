package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import org.firstinspires.ftc.teamcode.Robot.MecanumDrive;



@TeleOp(name="teleop")
public class TeleOpMode extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;
    Servo servo;

    public static final transient boolean FIELD_CENTRIC = true;

    @Override
    public void runOpMode() throws InterruptedException {
        // Main Drive
        MecanumDrive mecanumDrive = new MecanumDrive(
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                true
        );

        servo = hardwareMap.get(Servo.class, "clawservo");

        // Pulley
        DcMotorEx pulley = hardwareMap.get(DcMotorEx.class, "pulley");
        pulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pulley.setDirection(DcMotorSimple.Direction.REVERSE);
        pulley.setTargetPositionTolerance(40);
        // Reset encoder counts kept by motor
        pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boolean changed = true;

        // Gyroscope
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        float initHeading = angles.firstAngle;

        telemetry.addData("Status", "Initialized");
        //telemetry.addData("Initial Heading", initHeading);
        telemetry.update();

        waitForStart();


        // AFTER INIT:
        int pulleyDesiredPosition = 0;
        int pulleydiff;

        while(opModeIsActive()){
            //Get gyroscope angles
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double gyroAngle = angles.firstAngle;

            //Get the hypotenuse of the right triangle created by the position of the left gamepad stick on the x and y axis
            // Cubic Easing
            double r = Math.pow(Math.hypot(gamepad1.left_stick_y, -gamepad1.left_stick_x), 5);

            /*Get the angle that the left gamepad stick creates with the horizontal x axis.
            This angle is then offset by PI/4 so that when the stick is
            all the way in one direction the sin and cos are the same*/
            double angle = Math.atan2(-gamepad1.left_stick_x, gamepad1.left_stick_y) - Math.PI/4;

            if (FIELD_CENTRIC) {
                double cosA = Math.cos(gyroAngle);
                double sinA = Math.sin(gyroAngle);
                double gpX = gamepad1.left_stick_x;
                double gpY = gamepad1.left_stick_y;
                double X, Y;

                X = gpX * cosA - gpY * sinA;
                Y = gpX * sinA + gpY * cosA;


                angle = Math.atan2(X, Y) - (Math.PI/4);
            }


            /*Set the variables for the power of each of the motors to the inverse of either the cos or sin of the angle above,
            then multiply by the hypotenuse to get speed. The negative sign is there because the output of the gamepad
            is inverted.*/
            double fL = -Math.cos(angle)*r;
            double fR = -Math.sin(angle)*r;
            double bL = -Math.sin(angle)*r;
            double bR = -Math.cos(angle)*r;

            //Add or subtract the x value of the right stick to each of the motors so that the
            // robot can turn with the right stick.
            fL -= gamepad1.right_stick_x;
            fR += gamepad1.right_stick_x;
            bL -= gamepad1.right_stick_x;
            bR += gamepad1.right_stick_x;


            //Setting the power of the motors to the variables that were created earlier in this op mode.
            mecanumDrive.drive(fL, fR, bL, bR);

            // Left trigger -power, right trigger +power
            /*
            double pulleySpeed = gamepad1.right_trigger - gamepad1.left_trigger;
            pulley.setPower(pulleySpeed);
             */


            if        (gamepad1.a) { //ground
                pulleyDesiredPosition = 0;
                changed = true;
            } else if (gamepad1.b) { // low
                pulleyDesiredPosition = 1792;
                changed = true;
            } else if (gamepad1.x) { // medium
                pulleyDesiredPosition = 2836;
                changed = true;
            } else if (gamepad1.y) { // high
                pulleyDesiredPosition = 4175;
                changed = true;
            }

            pulley.setPower(0.5);
            pulleydiff = pulley.getCurrentPosition()-pulleyDesiredPosition;
            if (changed) {
                if (Math.abs(pulleydiff) > 100) {
                    pulley.setVelocity(-1000 * Math.tanh(0.025 * (pulleydiff)));
                } else if (Math.abs(pulleydiff) > 50){
                    pulley.setVelocity(Math.signum(pulleydiff)*100);
                } else {
                    pulley.setVelocity(0);
                    pulley.setPower(0);
                    changed = false;
                }
            } else {
                pulley.setVelocity(0);
                pulley.setPower(0);
            }

            //pulley.setTargetPosition(pulleyDesiredPosition);


            //pulleydiff = Math.abs(pulley.getCurrentPosition()-pulleyDesiredPosition);

//            if ( opModeIsActive() && pulley.isBusy()
//            && ( (pulley.getCurrentPosition() > pulleyDesiredPosition + 40) || (pulley.getCurrentPosition() < pulleyDesiredPosition - 40) )
//            ) {
//                pulley.setPower(0.25);
//            } else {
//                pulley.setPower(0);
//            }


            telemetry.addData("position", pulley.getCurrentPosition() + "  busy=" + pulley.isBusy());
            telemetry.addData("difference", pulley.getCurrentPosition()-pulleyDesiredPosition);





            // opening/closing of CRServo
/*
            if (gamepad1.left_bumper){
                servo.setPower(-1);
            } else if (gamepad1.right_bumper){
                servo.setPower(1);
            } else {
                servo.setPower(0);
            }
*/
            if (gamepad1.dpad_down) { //close
                servo.setPosition(0.0);
            } else if (gamepad1.dpad_up) { //open
                servo.setPosition(0.5);
            }

            telemetry.addData("Velocities\n", mecanumDrive.getVelocity() + "\n Pulley " + pulley.getVelocity());
            telemetry.addData("r", r);
            telemetry.addData("Assigned Servo position", servo.getPosition());

            /*
            telemetry.addData("Heading", angles.firstAngle);
            telemetry.addData("Roll", angles.secondAngle);
            telemetry.addData("Pitch", angles.thirdAngle);

            telemetry.addData("Angle (Rad) ", angles.firstAngle);
            telemetry.addData("Angle (Deg) ", angles.firstAngle*(180/Math.PI));
            */

            telemetry.addData("Power: ", "fL: " + fL + " fR: " + fR + " bL: " + bL + " bR: " + bR);

            telemetry.update();
        }


    }

}
