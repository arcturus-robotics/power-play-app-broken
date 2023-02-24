package org.firstinspires.ftc.teamcode.drive.opmode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveV2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
public class TrajectorySequenceTestingV2 extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    private DcMotorEx lift_Right;
    private DcMotorEx lift_Left;
    private DcMotorEx horizontal_slide;
    private Servo lclaw;
    private Servo rclaw;


    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double lclaw_open = 0.41;
    double rclaw_open = 0.25;
    double lclaw_closed = 0.5+0.045;
    double rclaw_closed = 0.15-0.045;

    int horizTargetPos = 0;


    // UNITS ARE METERS
    double tagsize = 0.166;
    int ground = 0;

    int IDTOI1 = 0;
    int IDTOI2 = 1;
    int IDTOI3 = 2;// Tag ID 18 from the 36h11 family

    int liftTargetPos = ground;

    AprilTagDetection tagOfInterest = null;

    boolean tagNotDetected = false;

    Pose2d startinglocatiion = new Pose2d(31 - 0.125, -64.28125, Math.toRadians(90));

    @Override
    public void runOpMode()
    {
        SampleMecanumDriveV2 drive = new SampleMecanumDriveV2(hardwareMap);
        drive.setPoseEstimate(startinglocatiion);

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



//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//            }
//        });

        telemetry.setMsTransmissionInterval(50);
        //adb connect 192.168.43.1:5555
        //very edge of tile on the left edge
        while (!isStarted() && !isStopRequested()) {
        }
        TrajectorySequence highcone1 = drive.trajectorySequenceBuilder(startinglocatiion)
                //.splineTo(new Vector2d(-35.42, -36.21), Math.toRadians(90.00))
                //.splineTo(new Vector2d(25.79+2.25, -2.74), Math.toRadians(140.0
                .strafeRight(4)
                .forward(55)
                .turn(Math.toRadians(-52))
                //.forward(30)
                //.turn(45)
                //.forward(10)
                .build();
    }
}