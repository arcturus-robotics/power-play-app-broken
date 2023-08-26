package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveV2;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous
public class PPAbsAuto2Cone_Right extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    private DcMotorEx lift;
    private Servo claw;
    private DistanceSensor leftdist, rightdist,frontdist;

    double half_bot = 10.375/2;
    double odowidth = 2.75;
    double distsenswidth = 0.65;
    double numloops = 0;
    double distsum = 0;
    double distavg = 0;
    double currentdistvalue;

    double startx = 24+half_bot;
    double starty = -72+8.5;

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

    //Pose2d startinglocatiion = new Pose2d(37.6, -64.28125, Math.toRadians(90));
    Pose2d startinglocatiion = new Pose2d(48-half_bot-odowidth, starty, Math.toRadians(90));
    //Pose2d startinglocatiion = new Pose2d(37.6, -64.28125, Math.toRadians(90));
    @Override
    public void runOpMode() {
        SampleMecanumDriveV2 drive = new SampleMecanumDriveV2(hardwareMap);
        drive.setPoseEstimate(startinglocatiion);

        lift = hardwareMap.get(DcMotorEx.class, "leftShooter");
        claw = hardwareMap.get(Servo.class, "claw");

        /*
        leftdist = hardwareMap.get(DistanceSensor.class, "sensor_range_left");
        rightdist = hardwareMap.get(DistanceSenkzsswwwwwwwwwwww wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwsor.class, "sensor_range_right");
        frontdist=hardwareMap.get(DistanceSensor.class, "sensor_range_front");
         */

        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.9);

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

        TrajectorySequence goingtohigh1 = drive.trajectorySequenceBuilder(startinglocatiion)

                .lineToConstantHeading(new Vector2d(startx+7.5,starty))
                .lineToConstantHeading(new Vector2d(startx+7.5,starty+50))
                .lineToConstantHeading(new Vector2d(24+1.3125,starty+50))
                .build();

        TrajectorySequence scoringposhigh1 = drive.trajectorySequenceBuilder(goingtohigh1.end())
                .lineToConstantHeading(new Vector2d(24+1.3125-0.5513245,-5+0.1))
                //.forward(7)
                .build();

        TrajectorySequence leavehigh1 = drive.trajectorySequenceBuilder(scoringposhigh1.end())
                .lineToConstantHeading(new Vector2d(24+1.3125-0.5513245, -14))
                .build();

        //parking
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(leavehigh1.end())
                .lineToConstantHeading(new Vector2d(startx+7.5+2, -13))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(leavehigh1.end())
                .lineToConstantHeading(new Vector2d(12, -14))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(leavehigh1.end())
                .lineToConstantHeading(new Vector2d(60, -13))
                .build();


        while (!isStarted() && !isStopRequested()) {

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
            telemetry.update();
            sleep(20);

            /*
            currentdistvalue = rightdist.getDistance(DistanceUnit.INCH);
            distsum += currentdistvalue;
            numloops += 1;
            telemetry.addData("DistValue", currentdistvalue);
            telemetry.update();
             */
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

        //distavg = distsum/numloops;
        telemetry.addData("DistAvg", distavg);
        telemetry.update();

        /*
        startinglocatiion = new Pose2d(72-half_bot-distavg-distsenswidth, -72+8.5, Math.toRadians(90));
        highcone1 = drive.trajectorySequenceBuilder(startinglocatiion)
                .lineToConstantHeading(new Vector2d(72-half_bot-distavg-distsenswidth-3,-72+8.5))
                .lineToConstantHeading(new Vector2d(72-half_bot-distavg-distsenswidth-3,-72+8.5+50))
                .lineToConstantHeading(new Vector2d(72-half_bot-distavg-distsenswidth-3-11.5,-72+8.5+50))
                .build();

         */

        drive.followTrajectorySequence(goingtohigh1);

        lift.setTargetPosition(4400);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.9);
        sleep(3000);
        drive.followTrajectorySequence(scoringposhigh1);
        lift.setTargetPosition(4000);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.9);
        sleep(500);
        claw.setPosition(0.68);
        drive.followTrajectorySequence(leavehigh1);
        lift.setTargetPosition(850);
        lift.setPower(0.9);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        try {
            if (tagOfInterest.id == IDTOI1) {
                drive.followTrajectorySequence(park1);

            } else if (tagOfInterest.id == IDTOI2) {
                drive.followTrajectorySequence(park2);
            } else {
                drive.followTrajectorySequence(park3);
            }
        }
        catch (Exception e){
            drive.followTrajectorySequence(park2);
        }
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