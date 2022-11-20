package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.ArcturusDriveNoRR;

@Autonomous(group = "drive")
public class OneConeAutoL extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    private static final Pose2d ORIGIN = new Pose2d(-63.0, -56.0, 0.0);

    private ArcturusDriveNoRR drive;
    private double number;
    private DcMotorEx lift,leftFront, leftRear, rightRear, rightFront;
    private Servo claw;
    private long freightboxdelay;
    double WorkingMotorMax = 0.6825;
    // private ArcturusVision vision;

    @Override
    public void runOpMode() {
        number = 0.5;
        // Dri
        drive = new ArcturusDriveNoRR(hardwareMap);
        //
        // intaketilt = hardwareMap.get(Servo.class, "ringpusher");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(0.9);

        //leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("sdjflkasfjsdlaf", "press play or you have 5 seconds to live");
        telemetry.update();
        waitForStart();


        //intaketilt.setPosition(0.58);

        //moving right to our substation
        drive.goingRight(1750, 0.1);

        //moving forward
        drive.goingForward(1750, 0.1);

        //moving a small bit right
        drive.goingRight(750, 0.1);

        //lift  up
        lift.setTargetPosition(4400);
        lift.setPower(0.5);

        //go forward to score
        drive.goingForward(1000, 0.1);

        //open claw
        claw.setPosition(0.65);
        sleep(1000);

        //go backward
        drive.goingBackward(1000, 0.1);

        //lift down
        lift.setTargetPosition(0);
        lift.setPower(0.5);

        //moving a small bit left
        drive.goingLeft(1000, 0.1);

        //go backward into substation
        drive.goingBackward(3000, 0.1);

        }


    }

