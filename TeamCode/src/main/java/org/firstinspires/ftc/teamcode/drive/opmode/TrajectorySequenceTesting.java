package org.firstinspires.ftc.teamcode.drive.opmode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.trajectory.DisplacementMarker;
import com.acmerobotics.roadrunner.trajectory.TemporalMarker;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;


@Autonomous
public class TrajectorySequenceTesting extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    private DcMotorEx lift;
    private Servo claw;


    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int IDTOI1 = 0;
    int IDTOI2 = 1;
    int IDTOI3 = 2;// Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    boolean tagNotDetected = false;

    Pose2d startinglocatiion = new Pose2d(31 - 0.125, -64.28125, Math.toRadians(90));

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startinglocatiion);

        lift =  hardwareMap.get(DcMotorEx.class, "leftShooter");
        claw = hardwareMap.get(Servo.class, "claw");

        lift.setTargetPosition(0);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        claw.setPosition(0.9);


        telemetry.setMsTransmissionInterval(50);
        //adb connect 192.168.43.1:5555
        //very edge of tile on the left edge
        TrajectorySequence highcone1 = drive.trajectorySequenceBuilder(startinglocatiion)
                //.splineTo(new Vector2d(-35.42, -36.21), Math.toRadians(90.00))
                //.splineTo(new Vector2d(25.79+2.25, -2.74), Math.toRadians(140.0
                .strafeRight(4)
                .forward(55)
                //.forward(30)
                //.turn(45)
                //.forward(10)
                .build();

    drive.followTrajectorySequence(highcone1);
    }
}