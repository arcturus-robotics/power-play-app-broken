package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private double leftFront = 0;
    private double leftRear = 0;
    private double rightFront =0;
    private double rightRear=0;

    @Override

    public void init() {
        drive = new ArcturusDriveNoRR(hardwareMap);
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
        drive.setMotorPowers(leftFront,leftRear,rightFront,rightRear);
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopped..");
    }


}