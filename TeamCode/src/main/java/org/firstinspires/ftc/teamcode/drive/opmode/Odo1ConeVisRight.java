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
public class Odo1ConeVisRight extends LinearOpMode
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

        TrajectorySequence highcone1 = drive.trajectorySequenceBuilder(startinglocatiion)
                .splineTo(new Vector2d(35.42, -36.21), Math.toRadians(90.00))
                //.splineTo(new Vector2d(25.79+2.25, -2.74), Math.toRadians(140.00))
                .splineTo(new Vector2d(26.9, -6.9289), Math.toRadians(138))
                /*
                Editing to make it align
                 */
                /*
                .back(3)
                .forward(2)
                .strafeRight(1)

                 */
                /*
                lowering lift
                 */

                .build();

        TrajectorySequence leavehighjunction = drive.trajectorySequenceBuilder(highcone1.end())
                .back(12)
                .build();

        TrajectorySequence parkingzone1 = drive.trajectorySequenceBuilder(leavehighjunction.end())
                .turn(Math.toRadians(42))
                .forward(24)
                .build();

        TrajectorySequence parkingzone2 = drive.trajectorySequenceBuilder(leavehighjunction.end())
                //.lineToLinearHeading(new Pose2d(36.42, -24.00, Math.toRadians(270)))
                .back(2)
                .turn(Math.toRadians(-48))
                .back(24)
                .build();

        TrajectorySequence parkingzone3 = drive.trajectorySequenceBuilder(leavehighjunction.end())
                //.lineToLinearHeading(new Pose2d(36.42, -24, Math.toRadians(90)))
                //.turn(Math.toRadians(90-drive.getPoseEstimate().getHeading()))
                //.back(40)
                //.strafeRight(25)
                //.forward(40)
            //  .splineTo(new Vector2d(48.11, -11.37), Math.toRadians(0))
                //.setReversed(true)
                .back(2)
                .turn(Math.toRadians(-138))
                .forward(20)
                .build();

        TrajectorySequence getcone = drive.trajectorySequenceBuilder(leavehighjunction.end())
                .lineToLinearHeading(new Pose2d(58.11, -11.58, Math.toRadians(190)))

                .build();


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

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

            telemetry.update();
            sleep(20);
        }


        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

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

        lift.setTargetPosition(4600);
        lift.setPower(0.9);
        drive.followTrajectorySequence(highcone1);
        sleep(10000);
        /*
        lift.setTargetPosition(0);
        lift.setPower(0.9);
        sleep(10000);
         */
        claw.setPosition(0.7);

        drive.followTrajectorySequence(leavehighjunction);
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