package org.firstinspires.ftc.teamcode.drive.opmode;

// import dandroid.text.style.TabStopSpan;

// import com.acmerobotics.roadrunner.geometry.Pose2d;
// import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.drive.ArcturusDriveNoRR;


import java.util.ArrayList;

import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * A teleop for use with a single controller.
 * Of course, it still works with two controllers plugged in,
 * but it will only use one of them.
 */
@TeleOp
public class PPTele2Control extends OpMode {
    // variables are set

    private ArcturusDriveNoRR drive;
    private DcMotorEx lift;
    private DcMotorEx lf;
    private DcMotorEx rf;
    private DcMotorEx rr;
    private DcMotorEx lr;
    private Servo claw;
    long delay = 0;

//    TouchSensor touch;

    double clawClosed = 0.98;
    double clawOpen = 0.74;
    double liftpos;

    boolean PID = true;
    boolean upsies;
    boolean motortoggle = false;
   boolean slowspeed = false;


    /*int high = 7900; for previous spool
    int medium = 5800; for previous spool
    int low = 3550; for previous spool*/
    // 4:7 = circumference of previous spool : circumference of new spool
    int maxheight = 4400;
    int high = 4400;
    int medium = 3200;
    int low = 1850;
    int ground = 0;
    int caldera = 750;
    int selectedpos = 0;
    double WorkingMotorMax = 0.6825-0.05;
    // 0.5 normal
    double MotorMaxSpeed = 0.8;
    int targetpos = ground;
    //double rightfrontpos,leftfrontpos,leftbackpos,rightbackpos;

    ArrayList<Boolean> boolArray = new ArrayList<Boolean>();
    int booleanIncrement = 0;

    @Override

    public void init() {
        drive = new ArcturusDriveNoRR(hardwareMap);

        //  noodle = hardwareMap.get(DcMotorEx.class, "intake");

        lift = hardwareMap.get(DcMotorEx.class, "leftShooter");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setTargetPosition(targetpos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.9);
        //noodle = hardwareMap.get(DcMotorEx.class,"rightShooter");

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(0.9);

//        touch = hardwareMap.get(TouchSensor.class, "Touch");

        //  lf = hardwareMap.get(DcMotorEx.class, "leftFront");
        //rr = hardwareMap.get(DcMotorEx.class, "rightRear");
        //lr = hardwareMap.get(DcMotorEx.class, "leftRear");
        //rf = hardwareMap.get(DcMotorEx.class, "rightFront");


    }

    @Override
    public void start() {
        telemetry.addData("Status", "STARTED!!");
    }


    @Override
    public void loop() {

        double leftFront = Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
        double leftRear = Range.clip(gamepad1.left_stick_y + gamepad1.right_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
        double rightRear = -Range.clip(gamepad1.right_stick_y - gamepad1.left_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
        double rightFront = -Range.clip(gamepad1.right_stick_y + gamepad1.right_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
//        if(gamepad1.right_bumper && motortoggle==false){
//            motortoggle = true;
//        }
//        else if(gamepad1.right_bumper && motortoggle==true){
//            motortoggle = false;
//        }
//        else {
//
//        }
        //touch sensor
//        if (touch.isPressed()) {
//          //  (motor).setPower(0);
//        } else { // Otherwise, run the motor
//          //  (motor).setPower(0.3);
//        }

        if (leftFront != 0 || leftRear != 0 || rightRear != 0 || rightFront != 0) {
            if (gamepad1.left_bumper){
                slowspeed = true;
                drive.setMotorPowers(leftFront*0.4, leftRear*0.4, rightFront*0.4, rightRear*0.4);
            }
//            else if (motortoggle) {
//                slowspeed = true;
//                drive.setMotorPowers(leftFront*0.4, leftRear*0.4, rightFront*0.4, rightRear*0.4);
//            }
            else {
                drive.setMotorPowers(leftFront, leftRear, rightFront, rightRear);
                slowspeed=false;
            }
        } else {
            if (gamepad1.right_trigger != 0) {
                drive.setMotorPowers(-0.7, 0.7, -0.7, 0.7);
            } else if (gamepad1.left_trigger != 0){
                drive.setMotorPowers(0.7, -0.7, 0.7, -0.7);
            }
             else{
                 drive.setMotorPowers(0, 0, 0,0);
             }
        }

        liftpos = lift.getCurrentPosition();

        // claw set and lifted to a position
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
                PID=true;
                targetpos=selectedpos;
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
            delay=System.nanoTime();
            claw.setPosition(0.9);
        } else if (gamepad2.x) {
            selectedpos=medium;
            upsies = true;
            delay=System.nanoTime();
            claw.setPosition(0.9);
        } else if (gamepad2.y) {
            selectedpos=high;
            upsies = true;
            delay=System.nanoTime();
            claw.setPosition(0.9);
        } else if (gamepad2.b) {
            upsies = false;
            //claw.setPosition(1);
            targetpos = ground;
            PID = true;

        } else if (gamepad2.dpad_left){
            upsies = false;
            targetpos = caldera;
            PID = true;
        }


        if (gamepad2.right_bumper) {
            claw.setPosition(clawClosed);
            // intaketilt.setPosition(0);
        } else if (gamepad2.left_bumper) {
            claw.setPosition(clawOpen);
            // intaketilt.setPcosition(1);
        }
        /*
        else {
            //intaketilt.setPower(0);5
        }*/

        if (gamepad2.share) {
            PID = false;
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


        // telemetry.addData("duckspeed",duckspeed);
        telemetry.addData("lift stay in place", PID);
        telemetry.addData("lift pos", liftpos);
//        telemetry.addData("Steve", 999);
//        telemetry.addData("Is toggled?", motortoggle);
        telemetry.addData("Is slow?", slowspeed);

        // telemetry.addData("right front", rightfrontpos);
        //telemetry.addData("right rear", rightbackpos);
        //telemetry.addData("left front", leftfrontpos);
        //telemetry.addData("left rear", leftbackpos );
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