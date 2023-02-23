package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.drive.ArcturusDriveNoRR;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



import java.util.ArrayList;

import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * A teleop to test motors
 */
@TeleOp
public class PPMotorTest extends OpMode {
    // variables are set

    private ArcturusDriveNoRR drive;
    private DcMotorEx lift_Right, lift_Left, horizontal_slide;
    private double leftFront = 0;
    private double leftRear = 0;
    private double rightFront =0;
    private double rightRear=0;

    private double RLpower=0;
    private double LLpower=0;
    private double HSpower=0;



    @Override

    public void init() {
        drive = new ArcturusDriveNoRR(hardwareMap);
        lift_Right = hardwareMap.get(DcMotorEx.class, "rightShooter");

        lift_Left= hardwareMap.get(DcMotorEx.class, "leftShooter");

        lift_Left.setDirection(DcMotorSimple.Direction.REVERSE);

        horizontal_slide = hardwareMap.get(DcMotorEx.class, "horizSlide");
    }

    @Override
    public void start() {
        telemetry.addData("Status", "STARTED!!");
    }


    @Override
    public void loop() {
        if (gamepad1.left_trigger!=0) {
            leftFront=1;
        }
        else {
            leftFront=0;
        }
        if (gamepad1.left_bumper) {
            leftRear=1;
        }
        else {
            leftRear=0;
        }
        if (gamepad1.right_trigger!=0) {
            rightFront =1;
        }
        else {
            rightFront=0;
        }
        if (gamepad1.right_bumper) {
            rightRear=1;
        }
        else {
            rightRear =0;
        }
        if (gamepad1.y) {
            HSpower=1;
        }
        else {
            HSpower =0;
        }
        if (gamepad1.x) {
            LLpower=1;
        }
        else {
            LLpower =0;
        }
        if (gamepad1.b) {
            RLpower=1;
        }
        else {
            RLpower =0;
        }

        drive.setMotorPowers(leftFront,leftRear,rightFront,rightRear);
        lift_Left.setPower(LLpower);
        lift_Right.setPower(RLpower);
        horizontal_slide.setPower(HSpower);
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopped..");
    }


}