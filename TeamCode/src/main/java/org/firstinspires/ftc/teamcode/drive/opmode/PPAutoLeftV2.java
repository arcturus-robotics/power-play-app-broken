package org.firstinspires.ftc.teamcode.drive.opmode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveV2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
public class PPAutoLeftV2 extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    private DcMotorEx lift_Right,lift_Left,horizontal_slide;
    private Servo lclaw, rclaw;
    private DistanceSensor liftsensor;


    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    int maxheight = 4400;
    int highj = 4020;
    int medj = 2940;
    int lowj = 1820;
    int nodej = 750;
    int ground = 0;
    double lclawpos, rclawpos;
    int liftSelectedPos = 0;
    int scoringPos = 1900;
    int intakePos = 0;
    int horizSelectedPos = 0;
    int horizTargetPos = intakePos;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int IDTOI1 = 0;
    int IDTOI2 = 1;
    int IDTOI3 = 2;// Tag ID 18 from the 36h11 family
    double lclaw_open = 0.41;
    double rclaw_open = 0.25;
    double lclaw_closed = 0.5+0.045;
    double rclaw_closed = 0.15-0.045;

    AprilTagDetection tagOfInterest = null;

    boolean tagNotDetected = false;
    int liftTargetPos = ground;

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

        liftsensor = hardwareMap.get(DistanceSensor.class, "liftDist");
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
                .forward(50)
                .turn(Math.toRadians(-35))
                //.forward(30)
                //.turn(45)
                //.forward(10)
                .build();
        TrajectorySequence highcone2 = drive.trajectorySequenceBuilder(highcone1.end())
                .turn(Math.toRadians(-(90-35)))
                .back(27)
                .build();
        TrajectorySequence highcone3 = drive.trajectorySequenceBuilder(highcone2.end())
                .forward(8)
                .turn(Math.toRadians(-60))
                .build();
        drive.followTrajectorySequence(highcone1);
        liftSelectedPos = highj;
        horizSelectedPos = scoringPos;
        sleep(5000);
        lclaw.setPosition(lclaw_open);
        rclaw.setPosition(rclaw_open);
        sleep(500);
        liftSelectedPos=lowj-1220;
        horizSelectedPos=intakePos;
        drive.followTrajectorySequence(highcone2);
        lclaw.setPosition(lclaw_closed);
        rclaw.setPosition(rclaw_closed);
        sleep(500);
        liftSelectedPos=lowj;
        horizSelectedPos=scoringPos;
        sleep(5000);
        drive.followTrajectorySequence(highcone3);
        lclaw.setPosition(lclaw_open);
        rclaw.setPosition(rclaw_open);
        sleep(500);
        liftSelectedPos=lowj-300;
        horizSelectedPos=intakePos;
    }
}