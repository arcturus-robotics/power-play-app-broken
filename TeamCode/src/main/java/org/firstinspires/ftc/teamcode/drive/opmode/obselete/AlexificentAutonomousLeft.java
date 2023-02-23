package org.firstinspires.ftc.teamcode.drive.opmode.obselete;
import org.firstinspires.ftc.teamcode.drive.opmode.AprilTagDetectionPipeline;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous
@Disabled
public class AlexificentAutonomousLeft extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    private DcMotorEx lift;
    private Servo claw;
    private TouchSensor touch_sensor;


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

    TrajectorySequence idiot;

    double LeftDist;
    double RightDist;
    double Half_bot;
    double needposx;

    private DistanceSensor sensorRange_left;
    private DistanceSensor sensorRange_right;

    AprilTagDetection tagOfInterest = null;

    boolean tagNotDetected = false;

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        lift =  hardwareMap.get(DcMotorEx.class, "leftShooter");
        claw = hardwareMap.get(Servo.class, "claw");

        sensorRange_left = hardwareMap.get(DistanceSensor.class, "sensor_range_left");
        sensorRange_right = hardwareMap.get(DistanceSensor.class, "sensor_range_right");

        Half_bot = 8;
        needposx = -40;

        lift.setTargetPosition(0);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        claw.setPosition(0.9);

        touch_sensor = hardwareMap.get(TouchSensor.class, "touch");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        //adb connect 192.168.43.1:5555
        //very edge of tile on the right edge

        LeftDist = sensorRange_left.getDistance(DistanceUnit.INCH);
        RightDist = sensorRange_right.getDistance(DistanceUnit.INCH);

        Pose2d startinglocatiion = new Pose2d(-72 + LeftDist + Half_bot, -64.28125, Math.toRadians(90));
        drive.setPoseEstimate(startinglocatiion);

        telemetry.setMsTransmissionInterval(50);
        //adb connect 192.168.43.1:5555
        //very edge of tile on the left edge

        TrajectorySequence beginning = drive.trajectorySequenceBuilder(startinglocatiion)
                .splineTo(new Vector2d(needposx, -64.28125), Math.toRadians(90.00))
                .build();
        TrajectorySequence highcone1 = drive.trajectorySequenceBuilder(beginning.end())
                //.splineTo(new Vector2d(-35.42, -36.21), Math.toRadians(90.00))
                //.splineTo(new Vector2d(25.79+2.25, -2.74), Math.toRadians(140.0
                .strafeRight(4)
                .forward(55)
                .turn(Math.toRadians(-48))
                //.forward(30)
                //.turn(45)
                //.forward(10)
                .build();

        TrajectorySequence highcone2 = drive.trajectorySequenceBuilder(highcone1.end())
                .forward(8)
                .build();
        TrajectorySequence highcone3 = drive.trajectorySequenceBuilder(highcone2.end())
                .back(8)
                .build();
        TrajectorySequence highcone4 = drive.trajectorySequenceBuilder(highcone3.end())
                .turn(Math.toRadians(48))
                .back(2.9)
                .turn(Math.toRadians(90))
                .forward(30)
                .build();
        TrajectorySequence highcone5 = drive.trajectorySequenceBuilder(highcone4.end())
                .back(30)
                .turn(Math.toRadians(42))
                .forward(9)
                .build();
        TrajectorySequence highcone6 =drive.trajectorySequenceBuilder(highcone5.end())
                .back(9)
                .turn(Math.toRadians(-42))
                .forward(30)
                .build();
        TrajectorySequence highcone7 = drive.trajectorySequenceBuilder(highcone6.end())
                .back(9)
                .turn(Math.toRadians(-132))
                .build();

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == IDTOI1 || tag.id == IDTOI2 || tag.id == IDTOI3) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }
            telemetry.update();
            sleep(20);
        }

        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            try {
                telemetry.update();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
            boolean tagNotDetected = true;
        }

        try {
            if (tagOfInterest.id == IDTOI1) {
                TrajectorySequence idiot = drive.trajectorySequenceBuilder(highcone6.end())
                        .back(10)
                        .strafeLeft(25)
                        .build();

            } else if (tagOfInterest.id == IDTOI2) {
                TrajectorySequence idiot = drive.trajectorySequenceBuilder(highcone6.end())
                        .back(10)
                        .build();
            } else {
                TrajectorySequence idiot = drive.trajectorySequenceBuilder(highcone6.end())
                        .back(10)
                        .strafeRight(23)
                        .build();
            }
        }
        catch (Exception e){
            TrajectorySequence idiot = drive.trajectorySequenceBuilder(highcone6.end())
                    .back(10)
                    .build();
        }

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
        claw.setPosition(0.6);
        sleep(100);
        drive.followTrajectorySequence(highcone3);
        sleep(100);
        claw.setPosition(0.9);
        lift.setTargetPosition(850);
        lift.setPower(0.9);
        claw.setPosition(0.7);
        sleep(2000);
        drive.followTrajectorySequence(highcone4);
        sleep(100);
        claw.setPosition(0.9);
        lift.setTargetPosition(2000);
        lift.setPower(0.9);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);
        drive.followTrajectorySequence(highcone5);
        lift.setTargetPosition(100);
        lift.setPower(0.9);
        claw.setPosition(0.6);
        lift.setTargetPosition(700);
        lift.setPower(0.9);
        drive.followTrajectorySequence(highcone6);
        sleep(100);
        claw.setPosition(0.9);
        lift.setTargetPosition(2000);
        lift.setPower(0.9);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);
        drive.followTrajectorySequence(highcone5);
        lift.setTargetPosition(100);
        lift.setPower(0.9);
        claw.setPosition(0.6);
        lift.setTargetPosition(700);
        lift.setPower(0.9);
        drive.followTrajectorySequence(highcone7);

        drive.followTrajectorySequence(idiot);
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("touching", touch_sensor.isPressed()));
        telemetry.addLine(String.format("left distance", sensorRange_left.getDistance(DistanceUnit.INCH)));
        telemetry.addLine(String.format("right distance", sensorRange_right.getDistance(DistanceUnit.INCH)));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}