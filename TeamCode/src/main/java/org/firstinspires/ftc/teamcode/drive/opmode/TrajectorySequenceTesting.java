package org.firstinspires.ftc.teamcode.drive.opmode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveV2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
public class TrajectorySequenceTesting extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    private DcMotorEx lift;
    private Servo claw;
    private DistanceSensor leftdist, rightdist,frontdist;


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
    public void runOpMode() {
        SampleMecanumDriveV2 drive = new SampleMecanumDriveV2(hardwareMap);
        drive.setPoseEstimate(startinglocatiion);

        lift = hardwareMap.get(DcMotorEx.class, "leftShooter");
        claw = hardwareMap.get(Servo.class, "claw");

        leftdist = hardwareMap.get(DistanceSensor.class, "sensor_range_left");
        rightdist = hardwareMap.get(DistanceSensor.class, "sensor_range_right");
        frontdist=hardwareMap.get(DistanceSensor.class, "sensor_range_front");


        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.9);

        claw.setPosition(0.9);

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

        TrajectorySequence highcone2 = drive.trajectorySequenceBuilder(highcone1.end())
                .forward(12)
                //.forward(7)
                .build();
        TrajectorySequence highcone3 = drive.trajectorySequenceBuilder(highcone2.end())
                .back(11)
                .turn(Math.toRadians(52))
                .back(5.2)
                .turn(Math.toRadians(90))
                .forward(29)
                //.back(7)
                .build();
        TrajectorySequence smallScore = drive.trajectorySequenceBuilder(highcone3.end())
                .back(26.5)
                .turn(Math.toRadians(42))
                .forward(10)
                .build();

        drive.followTrajectorySequence(highcone1);
        lift.setTargetPosition(4400);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.9);
        sleep(3000);
        drive.followTrajectorySequence(highcone2);
        lift.setTargetPosition(4000);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.9);
        sleep(500);
        claw.setPosition(0.68);
        lift.setTargetPosition(850);
        lift.setPower(0.9);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.followTrajectorySequence(highcone3);
        claw.setPosition(0.9);
        sleep(1000);
        lift.setTargetPosition(1850);
        lift.setPower(0.9);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);
        drive.followTrajectorySequence(smallScore);
        claw.setPosition(0.68);
    }

    public int DistSensAvg(DistanceSensor dist) {
        int avg;
        return 0;
    }
}