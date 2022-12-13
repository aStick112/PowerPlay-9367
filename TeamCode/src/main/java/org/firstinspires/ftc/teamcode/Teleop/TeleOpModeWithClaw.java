package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot.MecanumDrive;


@TeleOp(name="teleop EXPERIMENTAL- DONT RUN")
public class TeleOpModeWithClaw extends LinearOpMode {


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

        // Pulley
        DcMotorEx pulley = hardwareMap.get(DcMotorEx.class, "pulley");
        pulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo
        servo = hardwareMap.get(Servo.class, "servo");

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

        while(opModeIsActive()){
            //Get gyroscope angles
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double gyroAngle = angles.firstAngle;

            //Get the hypotenuse of the right triangle created by the position of the left gamepad stick on the x and y axis
            double r = Math.hypot(gamepad1.left_stick_y, -gamepad1.left_stick_x);

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

                if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) r = 1;
                else r = Math.hypot(gamepad1.left_stick_y, -gamepad1.left_stick_x);

                if      (gamepad1.dpad_up)    { gpX =  0; gpY = -1; }
                else if (gamepad1.dpad_down)  { gpX =  0; gpY =  1; }
                else if (gamepad1.dpad_left)  { gpX = -1; gpY =  0; }
                else if (gamepad1.dpad_right) { gpX =  1; gpY =  0; }

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

            // Left trigger down, right trigger pulley up
            double pulleySpeed = gamepad1.right_trigger - gamepad1.left_trigger;
            pulley.setPower(pulleySpeed);

            //Setting the power of the motors to the variables that were created earlier in this op mode.
            mecanumDrive.drive(fL, fR, bL, bR);

            // opening/closing of servo
            if (gamepad1.left_bumper){
                // open claw
                servo.setPosition(1);
            }
            if (gamepad1.right_bumper){
                // close claws
                servo.setPosition(0);
            }

            telemetry.addData("Velocities\n", mecanumDrive.getVelocity() + "\n Pulley " + pulley.getVelocity());

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
