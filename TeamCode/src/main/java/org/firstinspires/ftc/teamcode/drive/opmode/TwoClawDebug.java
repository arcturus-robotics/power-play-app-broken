package org.firstinspires.ftc.teamcode.drive.opmode;

// import dandroid.text.style.TabStopSpan;
// import com.acmerobotics.roadrunner.geometry.Pose2d;
// import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

/**
 * A teleop for use with a single controller.
 * Of course, it still works with two controllers plugged in,
 * but it will only use one of them.
 */
@TeleOp
public class TwoClawDebug extends OpMode {
    //private ArcturusDriveNoRR drive;
    private DcMotorEx lift_Right, lift_Left, horizontal_slide;
    private Servo lclaw, rclaw;
    //private Touch touch;

    long liftdelay = 0;
    long horizdelay = 0;

    double lclawpos, rclawpos;
    double liftpos_right, liftpos_left, horizpos;

    boolean liftPID = true;
    boolean horizPID = true;
    boolean drive_slowspeed = false;

    /*int high = 7900; for previous spool
    int medium = 5800; for previous spool
    int low = 3550; for previous spool*/
    // 4:7 = circumference of previous spool : circumference of new spool
    int maxheight = 4400;
    int highj = 4400;
    int medj = 3200;
    int lowj = 1850;
    int nodej = 750;
    int ground = 0;
    int liftSelectedPos = 0;
    int liftTargetPos = ground;

    int maxhoriz = 2000;
    int scoringPos = 1503;
    int intakePos = 0;
    int horizSelectedPos = 0;
    int horizTargetPos = intakePos;

    double lclaw_open = 0.41;
    double rclaw_open = 0.29;
    double lclaw_closed = 0.51+0.045;
    double rclaw_closed = 0.22-0.045;

    //double WorkingMotorMax = 0.6825-0.05;
    double MotorMaxSpeed = 0.8;

    ArrayList<Boolean> boolArray = new ArrayList<Boolean>();
    int booleanIncrement = 0;

    @Override

    public void init() {
        ///drive = new ArcturusDriveNoRR(hardwareMap);
        lift_Right = hardwareMap.get(DcMotorEx.class, "rightShooter");
        lift_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_Right.setTargetPosition(liftTargetPos);
        lift_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift_Right.setPower(1);

        lift_Left= hardwareMap.get(DcMotorEx.class, "leftShooter");
        lift_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_Left.setTargetPosition(liftTargetPos);
        lift_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift_Left.setPower(1);

        lift_Left.setDirection(DcMotorSimple.Direction.REVERSE);

        horizontal_slide = hardwareMap.get(DcMotorEx.class, "horizSlide");
        horizontal_slide .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontal_slide .setTargetPosition(horizTargetPos);
        horizontal_slide .setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontal_slide .setPower(0.9);


        lclaw = hardwareMap.get(Servo.class, "lclaw");
        rclaw = hardwareMap.get(Servo.class, "rclaw");

        lclaw.setPosition(lclaw_closed);
        rclaw.setPosition(rclaw_closed);

        // touch = hardwareMap.get(TouchSensor.class, "Touch");

    }

    @Override
    public void start() {
        telemetry.addData("Status", "STARTED!!");
    }


    @Override
    public void loop() {
        /*
        double leftFront = -Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
        double leftRear = -Range.clip(gamepad1.left_stick_y + gamepad1.right_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
        double rightRear = -Range.clip(gamepad1.right_stick_y - gamepad1.left_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
        double rightFront = -Range.clip(gamepad1.right_stick_y + gamepad1.right_stick_x, -MotorMaxSpeed, MotorMaxSpeed);


        if (leftFront != 0 || leftRear != 0 || rightRear != 0 || rightFront != 0) {
            if (gamepad1.left_bumper){
                drive_slowspeed = true;
                drive.setMotorPowers(leftFront*0.4, leftRear*0.4, rightFront*0.4, rightRear*0.4);
            }
            else {
                drive.setMotorPowers(leftFront, leftRear, rightFront, rightRear);
                drive_slowspeed =false;
            }
        } else {
            if (gamepad1.right_trigger != 0) {
                drive.setMotorPowers(0.7, -0.7, -0.7, 0.7);
            } else if (gamepad1.left_trigger != 0){
                drive.setMotorPowers(-0.7, 0.7, 0.7, -0.7);
            }
            else{
                drive.setMotorPowers(0, 0, 0,0);
            }
        }

         */

        // lift PID loop
        if (liftPID) {
            if (liftTargetPos > maxheight) {
                liftTargetPos = maxheight;
            }
            if (liftTargetPos < 0) {
                liftTargetPos = 0;
            }
            lift_Right.setTargetPosition(liftTargetPos);
            lift_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lift_Left.setTargetPosition(liftTargetPos);
            lift_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lift_Right.setPower(1);
            lift_Left.setPower(1);
        }

        //horizontal_slide PID loop
        if (horizPID) {
            if (horizTargetPos > maxhoriz) {
                horizTargetPos = maxhoriz;
            }
            if (horizTargetPos < 0) {
                horizTargetPos = 0;
            }
            horizontal_slide.setTargetPosition(horizTargetPos);
            horizontal_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            horizontal_slide.setPower(1);
        }

        //used for waiting for claw to close before lift up
        if (liftdelay != 0) {
            if (System.nanoTime() > 2e+8 + liftdelay) {
                liftdelay = 0;
                liftPID = true;
                liftTargetPos = liftSelectedPos;
                horizdelay = System.nanoTime();
            }
        }

        //waiting for lift to go up before making horizontal slide go forward
        if (horizdelay != 0) {
            if (System.nanoTime() > 5e+8 + horizdelay) {
                horizdelay = 0;
                horizPID = true;
                horizTargetPos = horizSelectedPos;
            }
        }

        //non-PID control for horizontal slide
        /*
        if (gamepad2.dpad_right) {
            horizPID = false;
            horizontal_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            horizontal_slide.setPower(1);
        } else if (gamepad2.dpad_left) {
            horizPID = false;
            horizontal_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            horizontal_slide.setPower(-1);
        } else if (!horizPID) {
            horizontal_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            horizontal_slide.setPower(0);
        }

         */

        if (gamepad2.dpad_left){
            horizTargetPos += -20;
        }
        else if (gamepad2.dpad_right){
            horizTargetPos += 20;
        }

        //non-PID control for lift
        if (gamepad2.dpad_up) {
            liftPID = false;
            lift_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift_Left.setPower(1);
            lift_Right.setPower(1);

            //lclaw.setPosition(lclaw_closed);
            //rclaw.setPosition(rclaw_closed);
        } else if (gamepad2.dpad_down) {
            liftPID = false;
            lift_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift_Left.setPower(-1);
            lift_Right.setPower(-1);
            //lclaw.setPosition(lclaw_closed);
            //rclaw.setPosition(rclaw_closed);
        } else if (!liftPID) {
            lift_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift_Left.setPower(0);
            lift_Right.setPower(0);
        }

        //non-preset PID control for lift
        if (gamepad2.right_trigger != 0) {
            liftTargetPos -= 20;
        } else if (gamepad2.left_trigger != 0) {
            liftTargetPos += 20;
        }

        //the following 4 if/else if statements tell the robot which stage of the lift
        //uses the right of the second controller with y referring to up, x referring to medium, a referring to down, and b referring to default
        //the preference goes y,x,a, and b from most powerful command to least powerful command
        if (gamepad2.a) {
            liftPID = true;
            horizPID = true;
            liftSelectedPos = lowj;
            horizSelectedPos = scoringPos;
            lclaw.setPosition(lclaw_closed);
            rclaw.setPosition(rclaw_closed);
            liftdelay =System.nanoTime();
        } else if (gamepad2.x) {
            liftPID = true;
            horizPID = true;
            liftSelectedPos = medj;
            horizSelectedPos = scoringPos;
            lclaw.setPosition(lclaw_closed);
            rclaw.setPosition(rclaw_closed);
            liftdelay =System.nanoTime();
        } else if (gamepad2.y) {
            liftPID = true;
            horizPID = true;
            liftSelectedPos = highj;
            horizSelectedPos = scoringPos;
            lclaw.setPosition(lclaw_closed);
            rclaw.setPosition(rclaw_closed);
            liftdelay =System.nanoTime();
        } else if (gamepad2.right_stick_button){
            liftPID = true;
            horizPID = true;
            liftSelectedPos = nodej;
            horizSelectedPos = scoringPos;
            lclaw.setPosition(lclaw_closed);
            rclaw.setPosition(rclaw_closed);
            liftdelay =System.nanoTime();
        } else if (gamepad2.b) {
            liftTargetPos = ground;
            liftPID = true;
        }

        //controlling claw
        if (gamepad2.right_bumper) {
            /*
            lclaw.setPosition(Range.clip(lclawpos+ 0.01, 0, 1));
            rclaw.setPosition(Range.clip(rclawpos+ 0.01, 0, 1));

             */


            lclaw.setPosition(lclaw_closed);
            rclaw.setPosition(rclaw_closed);


        } else if (gamepad2.left_bumper) {

            /*
            lclaw.setPosition(Range.clip(lclawpos - 0.01, 0, 1));
            rclaw.setPosition(Range.clip(rclawpos - 0.01, 0, 1));

             */

            lclaw.setPosition(lclaw_open);
            rclaw.setPosition(rclaw_open);
        }

        //failsafe to reset encoder
        if (gamepad2.share) {
            liftPID = false;
            lift_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        lclawpos = lclaw.getPosition();
        rclawpos = rclaw.getPosition();
        liftpos_right = lift_Right.getCurrentPosition();
        liftpos_left = lift_Left.getCurrentPosition();
        horizpos = horizontal_slide.getCurrentPosition();

        telemetry.addData("vertical stay in place", liftPID);
        telemetry.addData("lift pos right", liftpos_right);
        telemetry.addData("lift target", liftTargetPos);
        telemetry.addData("lift pos left", liftpos_left);
        telemetry.addData("horiz slide pos", horizpos);
        telemetry.addData("lclaw pos", lclawpos);
        telemetry.addData("rclaw pos", rclawpos);
        telemetry.addData("Is slow?", drive_slowspeed);
        telemetry.update();

        booleanIncrement = 0;
    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopped..");
    }

    private boolean ifpressed(boolean button) {
        boolean output = false;
        if (boolArray.size() == booleanIncrement) {
            boolArray.add(false);
        }
        boolean buttonWas = boolArray.get(booleanIncrement);
        if (button != buttonWas && button == true) {
            output = true;
        }
        boolArray.set(booleanIncrement, button);
        booleanIncrement += 1;
        return output;
    }

}