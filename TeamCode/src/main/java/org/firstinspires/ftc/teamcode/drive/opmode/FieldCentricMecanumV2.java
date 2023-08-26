package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentricMecanumV2 extends LinearOpMode {
    private IMU imu;
    //position for the lift's current position and the position we are targeting
    int targetpos = 0;
    double liftpos;
    int selectedpos = 0;
    //PID boolean
    boolean PID = true;
    //claw stages defined
    double clawClosed = 0.98;
    double clawOpen = 0.74;
    //boolean for to see if the robot's lift is going up.
    boolean upsies;
    //maxheight and stages for lift defined
    int maxheight = 4400;
    int high = 4400;
    int medium = 3200;
    int low = 1850;
    int ground = 0;
    int caldera = 750;
    //delay variable
    long delay = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftRear");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightRear");
        DcMotor lift = hardwareMap.dcMotor.get("leftShooter");
        Servo claw = hardwareMap.servo.get("claw");
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; //Remember, reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }
            //adb connect 192.168.43.1:5555
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                if (gamepad1.left_bumper) {
                    denominator*=(1/0.4);
                }
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;


            liftpos = lift.getCurrentPosition();

            if (PID) {
                if (targetpos > maxheight) {
                    targetpos = maxheight;
                }
                if (targetpos < 0) {
                    targetpos = 0;
                }
                lift.setTargetPosition(targetpos);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (upsies) {
                    lift.setPower(0.9);
                } else {
                    lift.setPower(0.9);
                }
            }
            if (delay != 0) {
                if (System.nanoTime() > 2e+8 + delay) {
                    delay = 0;
                    PID = true;
                    targetpos = selectedpos;
                }
            }

            if (gamepad2.dpad_up) {
                PID = false;
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                claw.setPosition(0.9);
                lift.setPower(0.9);

                //  noodle.setPower(-1);
            } else if (gamepad2.dpad_down) {
                PID = false;
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.setPower(-1);

                //noodle.setPower(1);
            } else if (!PID) {
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.setPower(0);
                //          lift.setTargetPosition(0);

            }

            if (gamepad2.right_trigger != 0) {
                targetpos -= 20;
                upsies = true;
            } else if (gamepad2.left_trigger != 0) {
                targetpos += 20;
                upsies = true;
            }


            //the following 4 if/else if statements tell the robot which stage of the lift
            //uses the right of the second controller with y referring to up, x referring to medium, a referring to down, and b referring to default
            //the preference goes y,x,a, and b from most powerful command to least powerful command
            if (gamepad2.a) {
                selectedpos = low;
                upsies = true;
                delay = System.nanoTime();
                claw.setPosition(0.9);
            } else if (gamepad2.x) {
                selectedpos = medium;
                upsies = true;
                delay = System.nanoTime();
                claw.setPosition(0.9);
            } else if (gamepad2.y) {
                selectedpos = high;
                upsies = true;
                delay = System.nanoTime();
                claw.setPosition(0.9);
            } else if (gamepad2.b) {
                upsies = false;
                //claw.setPosition(1);
                targetpos = ground;
                PID = true;

            } else if (gamepad2.dpad_left) {
                upsies = false;
                targetpos = caldera;
                PID = true;
            }


            if (gamepad2.right_bumper) {
                claw.setPosition(clawClosed);
                // intaketilt.setPosition(0);
            } else if (gamepad2.left_bumper) {
                claw.setPosition(clawOpen);}
                // intaketilt.setPcosition(1);

                motorFrontLeft.setPower(frontLeftPower);
                motorBackLeft.setPower(backLeftPower);
                motorFrontRight.setPower(frontRightPower);
                motorBackRight.setPower(backRightPower);

                telemetry.addData("Heading", botHeading);
                telemetry.update();

        }
    }
}