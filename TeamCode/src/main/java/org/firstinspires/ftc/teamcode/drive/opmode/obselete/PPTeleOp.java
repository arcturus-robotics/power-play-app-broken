package org.firstinspires.ftc.teamcode.drive.opmode.obselete;

// import dandroid.text.style.TabStopSpan;

// import com.acmerobotics.roadrunner.geometry.Pose2d;
// import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.ArcturusDriveRR;

import java.util.ArrayList;

/**
 * A teleop for use with a single controller.
 * Of course, it still works with two controllers plugged in,
 * but it will only use one of them.
 */
@Disabled
@TeleOp(group = "drive")
public class PPTeleOp extends OpMode {
    // variables are set
    private ArcturusDriveRR drive;
    private DcMotorEx duckwheel, lift, lf, rf, rr, lr;
    private Servo claw;
    double duckspeed = 0.55;
    double noodlespeed = 0.31;
    boolean noodlePower = false;
    boolean noodleintake = true;
    double clawpos;
    double liftpos;

    boolean PID = false;
    boolean upsies;
    long freightboxdelay = 0;
    double drivespeed = 3/4;

    /*int high = 7900; for previous spool
    int medium = 5800; for previous spool
    int low = 3550; for previous spool*/
    // 4:7 = circumference of previous spool : circumference of new spool
    int maxheight = 5100;
    int high = 4400;
    int medium = 3400;
    int low = 2250;
    int ground = 0;
    double WorkingMotorMax = 0.6825;
    int targetpos = 0;
    //double rightfrontpos,leftfrontpos,leftbackpos,rightbackpos;

    ArrayList<Boolean> boolArray = new ArrayList<Boolean>();
    int booleanIncrement = 0;

    @Override

    public void init() {
        drive = new ArcturusDriveRR(hardwareMap);
        //
        //  noodle = hardwareMap.get(DcMotorEx.class, "intake");
        duckwheel = hardwareMap.get(DcMotorEx.class, "front");

        lift =  hardwareMap.get(DcMotorEx.class, "leftShooter");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //noodle = hardwareMap.get(DcMotorEx.class,"rightShooter");

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(0.9);

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
        drive.update();

        double leftFront = -Range.clip(gamepad2.left_stick_y + gamepad2.left_stick_x, -1, 1);
        double leftRear = -Range.clip(gamepad2.left_stick_y - gamepad2.right_stick_x, -1, 1);
        double rightRear = -Range.clip(gamepad2.right_stick_y + gamepad2.left_stick_x, -1, 1);
        double rightFront = -Range.clip(gamepad2.right_stick_y - gamepad2.right_stick_x, -1, 1);
//        double leftFront = -Range.clip(gamepad2.left_stick_y + gamepad2.left_stick_x, -drivespeed, drivespeed);
//        double leftRear = -Range.clip(gamepad2.left_stick_y - gamepad2.right_stick_x, -drivespeed, drivespeed);
//        double rightRear = -Range.clip(gamepad2.right_stick_y + gamepad2.left_stick_x, -drivespeed, drivespeed);
//        double rightFront = -Range.clip(gamepad2.right_stick_y - gamepad2.right_stick_x, -drivespeed, drivespeed);

        //drive.setMotorPowers(leftFront*drivespeed, leftRear*drivespeed, rightRear*drivespeed, rightFront*drivespeed);
        drive.setMotorPowers(leftFront, leftRear, rightRear, rightFront);


        clawpos = claw.getPosition();
        liftpos = lift.getCurrentPosition();
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
                lift.setPower(1);
            }
            else {
                lift.setPower(0.5);
            }
        }


        if (gamepad2.dpad_up)
        {
            PID = false;
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setPower(1);

          //  noodle.setPower(-1);
        }
        else if (gamepad2.dpad_down) {
            PID = false;
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setPower(-1);

            //noodle.setPower(1);
        }
        else if (!PID)
        {
           lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           lift.setPower(0);
 //          lift.setTargetPosition(0);

        }

        if (gamepad2.right_trigger != 0) {
            targetpos += 20;
            upsies = true;
        }

        else if (gamepad2.left_trigger != 0) {
            targetpos -= 20;
            upsies = true;
        }



        //the following 4 if/else if statements tell the robot which stage of the lift
        //uses the right of the second controller with y referring to up, x referring to medium, a referring to down, and b referring to default
        //the preference goes y,x,a, and b from most powerful command to least powerful command
        if (gamepad2.a) {targetpos = low;
            upsies = true;
            PID = true;
        }

        else if (gamepad2.x) {targetpos = medium;
            upsies = true;
            PID = true;
        }

        else if (gamepad2.y) {targetpos = high;
            upsies = true;
            PID = true;
        }

        else if (gamepad2.b) {
            upsies = false;
            //claw.setPosition(1);
            targetpos = ground;
            PID = true;
        }

        if (gamepad2.right_bumper) {
            claw.setPosition( Range.clip(clawpos  + 0.02, 0.65, 0.9) );
            // intaketilt.setPosition(0);
        }
        else if (gamepad2.left_bumper) {
            claw.setPosition( Range.clip(clawpos  - 0.02, 0.65, 0.9) );
            // intaketilt.setPcosition(1);
        }
        /*
        else {
            //intaketilt.setPower(0);5
        }*/


        // telemetry.addData("duckspeed",duckspeed);
        telemetry.addData("lift stay in place", PID);
        telemetry.addData("lift pos", liftpos);
        telemetry.addData("claw pos", clawpos);
        telemetry.addData("Steve", 999);

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
