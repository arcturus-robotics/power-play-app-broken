package org.firstinspires.ftc.teamcode.drive.opmode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveV2;

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

import java.util.ArrayList;

import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous
@Disabled

public class TwoConeVisRight_NotWorking extends LinearOpMode
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

    double LeftDist;
    double RightDist;
    double Half_bot = 10.5/2;
    double distsenswidth = 0.65;
    double numloops = 0;
    double distsum = 0;
    double distavg = 0;

    private DistanceSensor sensorRange_left;
    private DistanceSensor sensorRange_right;

    AprilTagDetection tagOfInterest = null;

    boolean tagNotDetected = false;

    @Override
    public void runOpMode()
    {
        SampleMecanumDriveV2 drive = new SampleMecanumDriveV2(hardwareMap);

        lift =  hardwareMap.get(DcMotorEx.class, "leftShooter");
        claw = hardwareMap.get(Servo.class, "claw");

        sensorRange_left = hardwareMap.get(DistanceSensor.class, "sensor_range_left");
        sensorRange_right = hardwareMap.get(DistanceSensor.class, "sensor_range_right");


        lift.setTargetPosition(0);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        claw.setPosition(0.9);

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

        Pose2d startinglocatiion = new Pose2d(31 - 0.125, -64.28125, Math.toRadians(90));
        drive.setPoseEstimate(startinglocatiion);

        TrajectorySequence beginning = drive.trajectorySequenceBuilder(startinglocatiion)
                .strafeRight(4)
                .forward(55)
                .turn(Math.toRadians(-52))
                .build();
        TrajectorySequence highcone2 = drive.trajectorySequenceBuilder(beginning.end())
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

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == IDTOI1 || tag.id == IDTOI2 || tag.id == IDTOI3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }


            distsum += sensorRange_right.getDistance(DistanceUnit.INCH);
            numloops += 1;

            telemetry.update();
            sleep(30);
        }

        distavg = distsum/numloops;
        drive.setPoseEstimate(new Pose2d(72-distsenswidth-distavg, -72+8.5,  Math.toRadians(90)));

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

        lift.setTargetPosition(4400);
        lift.setPower(0.9);
        //drive.followTrajectorySequence(highcone1);
        sleep(10000);
        /*
        lift.setTargetPosition(0);
        lift.setPower(0.9);
        sleep(10000);
         */
        claw.setPosition(0.7);

        //drive.followTrajectorySequence(leavehighjunction);
        sleep(1000);

        //drive.followTrajectorySequence(getcone);
        // sleep(1000)

        //drive.followTrajectorySequence(highcone1)
        //sleep(1000)

        //drive.followTrajectorySequence(leavehighjunction);
        //sleep(1000)

        /*
        PARKING
         */
        /*
        try {
            if (tagOfInterest.id == IDTOI1) {
                drive.followTrajectorySequence(parkingzone1);

            } else if (tagOfInterest.id == IDTOI2) {
                drive.followTrajectorySequence(parkingzone2);
            } else {
                drive.followTrajectorySequence(parkingzone3);
            }
        }
        catch (Exception e){
            drive.followTrajectorySequence(parkingzone2);
        }

         */
        /*
        CODE FOR TAG NOT DETECTED
         */

        lift.setTargetPosition(0);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}