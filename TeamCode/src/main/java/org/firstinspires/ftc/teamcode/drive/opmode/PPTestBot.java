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

/**
 * A teleop for use with a single controller.
 * Of course, it still works with two controllers plugged in,
 * but it will only use one of them.
 */
@TeleOp
public class PPTestBot extends OpMode {
    // variables are set
    private ArcturusDriveNoRR drive;
    private DcMotorEx duckwheel;
    private DcMotorEx lift;
    private DcMotorEx lf;
    private DcMotorEx rf;
    private DcMotorEx rr;
    private DcMotorEx lr;
    private Servo claw;
    long delay = 0;

    double clawpos;
    double liftpos;

    boolean PID = true;
    boolean upsies;


    /*int high = 7900; for previous spool
    int medium = 5800; for previous spool
    int low = 3550; for previous spool*/
    // 4:7 = circumference of previous spool : circumference of new spool
    int maxheight = 5100;
    int high = 4400;
    int medium = 3200;
    int low = 2000;
    int ground = 80;
    int caldera = 750;
    int selectedpos = 0;
    double WorkingMotorMax = 0.6825;
    // 0.5 normal
    double MotorMaxSpeed = 0.65;
    int targetpos = ground;
    //double rightfrontpos,leftfrontpos,leftbackpos,rightbackpos;

    ArrayList<Boolean> boolArray = new ArrayList<Boolean>();
    int booleanIncrement = 0;

    @Override

    public void init() {
        drive = new ArcturusDriveNoRR(hardwareMap);

        //  noodle = hardwareMap.get(DcMotorEx.class, "intake");
//        duckwheel = hardwareMap.get(DcMotorEx.class, "front");
//
//        lift = hardwareMap.get(DcMotorEx.class, "leftShooter");
//        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lift.setTargetPosition(targetpos);
//        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lift.setPower(1);
//        //noodle = hardwareMap.get(DcMotorEx.class,"rightShooter");
//
//        claw = hardwareMap.get(Servo.class, "claw");
//        claw.setPosition(0.9);


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

        double leftFront = -Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
        double leftRear = -Range.clip(gamepad1.left_stick_y + gamepad1.right_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
        double rightRear = -Range.clip(gamepad1.right_stick_y - gamepad1.left_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
        double rightFront = -Range.clip(gamepad1.right_stick_y + gamepad1.right_stick_x, -MotorMaxSpeed, MotorMaxSpeed);

        if (leftFront != 0 || leftRear != 0 || rightRear != 0 || rightFront != 0) {
            drive.setMotorPowers(leftFront, leftRear, rightFront, rightRear);
        } else {
            if (gamepad1.right_trigger != 0) {
                drive.setMotorPowers(MotorMaxSpeed, -MotorMaxSpeed, -MotorMaxSpeed, MotorMaxSpeed);
            } else if (gamepad1.left_trigger != 0){
                drive.setMotorPowers(-MotorMaxSpeed, MotorMaxSpeed, MotorMaxSpeed, -MotorMaxSpeed);
            }
            else{
                drive.setMotorPowers(0, 0, 0,0);
            }
        }

//        clawpos = claw.getPosition();
//        liftpos = lift.getCurrentPosition();
        //liftpos = noodle.getCurrentPosition();

        //rightfrontpos = rf.getCurrentPosition();
        //rightbackpos = rr.getCurrentPosition();
        //leftbackpos = lr.getCurrentPosition();
        //leftfrontpos = lf.getCurrentPosition();


        /*
        if (gamepad1.x) {
            drive.setMotorPowers(1,0,0,0);
        }
        else if (gamepad1.y) {
            drive.setMotorPowers(0,0,0,1);
        }
        else if (gamepad1.a) {
            drive.setMotorPowers(0,1,0,0);
        }
        else if (gamepad1.b) {
            drive.setMotorPowers(0,0,1,0);
        }
        else if (gamepad1.dpad_left) {
            drive.setMotorPowers(1,-1,1,-1);
        }
        else if (gamepad1.dpad_right) {
            drive.setMotorPowers(-1,1,-1,1);
        }
        else {
            drive.setMotorPowers(0,0,0,0);
        }
        */


        // claw set and lifted to a position
//        if (PID) {
//            if (targetpos > maxheight) {
//                targetpos = maxheight;
//            }
//            if (targetpos < 0) {
//                targetpos = 0;
//            }
//            lift.setTargetPosition(targetpos);
//            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            if (upsies) {
//                lift.setPower(1);
//            } else {
//                lift.setPower(1);
//            }
//        }
//        if (delay != 0) {
//            if (System.nanoTime() > 2e+8 + delay) {
//                delay = 0;
//                PID=true;
//                targetpos=selectedpos;
//            }
//        }
//
//        if (gamepad2.dpad_up) {
//            PID = false;
//            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            claw.setPosition(0.9);
//            lift.setPower(1);
//
//            //  noodle.setPower(-1);
//        } else if (gamepad2.dpad_down) {
//            PID = false;
//            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            lift.setPower(-1);
//
//            //noodle.setPower(1);
//        } else if (!PID) {
//            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            lift.setPower(0);
//            //          lift.setTargetPosition(0);
//
//        }

//        if (gamepad2.right_trigger != 0) {
//            targetpos -= 20;
//            upsies = true;
//        } else if (gamepad2.left_trigger != 0) {
//            targetpos += 20;
//            upsies = true;
//        }


        //the following 4 if/else if statements tell the robot which stage of the lift
        //uses the right of the second controller with y referring to up, x referring to medium, a referring to down, and b referring to default
        //the preference goes y,x,a, and b from most powerful command to least powerful command
//        if (gamepad2.a) {
//            selectedpos = low;
//            upsies = true;
//            delay=System.nanoTime();
//            claw.setPosition(0.9);
//        } else if (gamepad2.x) {
//            selectedpos=medium;
//            upsies = true;
//            delay=System.nanoTime();
//            claw.setPosition(0.9);
//        } else if (gamepad2.y) {
//            selectedpos=high;
//            upsies = true;
//            delay=System.nanoTime();
//            claw.setPosition(0.9);
//        } else if (gamepad2.b) {
//            upsies = false;
//            //claw.setPosition(1);
//            targetpos = ground;
//            PID = true;
//
//        } else if (gamepad2.dpad_left){
//            upsies = false;
//            targetpos = caldera;
//            PID = true;
//        }
//
//
//        if (gamepad2.right_bumper) {
//            claw.setPosition(Range.clip(clawpos + 0.02, 0.7, 0.9));
//            // intaketilt.setPosition(0);
//        } else if (gamepad2.left_bumper) {
//            claw.setPosition(Range.clip(clawpos - 0.02, 0.7, 0.9));
//            // intaketilt.setPcosition(1);
//        }
        /*
        else {
            //intaketilt.setPower(0);5
        }*/

//        if (gamepad2.share) {
//            PID = false;
//            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }


//        // telemetry.addData("duckspeed",duckspeed);
//        telemetry.addData("lift stay in place", PID);
//        telemetry.addData("lift pos", liftpos);
//        telemetry.addData("claw pos", clawpos);
//        telemetry.addData("Steve", 999);

//         telemetry.addData("right front", rightfrontpos);
//        telemetry.addData("right rear", rightbackpos);
//        telemetry.addData("left front", leftfrontpos);
//        telemetry.addData("left rear", leftbackpos );
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