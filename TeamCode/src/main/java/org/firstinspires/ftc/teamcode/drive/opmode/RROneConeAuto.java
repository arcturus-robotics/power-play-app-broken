package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive")
public class RROneConeAuto extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    private static final Pose2d ORIGIN = new Pose2d(-63.0, -56.0, 0.0);

    private SampleMecanumDrive drive;
    private double number;
    private DcMotorEx lift,leftFront, leftRear, rightRear, rightFront;
    private Servo claw;
    private long freightboxdelay;
    double WorkingMotorMax = 0.6825;
    // private ArcturusVision vision;

    @Override
    public void runOpMode() {
        number = 0.5;
        // Drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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


        //moving forward


        //moving a small bit right


        //lift  up
        lift.setTargetPosition(4400);
        lift.setPower(0.5);

        //go forward to score


        //open claw
        claw.setPosition(0.65);
        sleep(1000);

        //go backward


        //lift down
        lift.setTargetPosition(0);
        lift.setPower(0.5);

        //moving a small bit left


        //go backward into substation


    }


}

