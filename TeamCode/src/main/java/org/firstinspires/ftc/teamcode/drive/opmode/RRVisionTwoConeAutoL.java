package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import java.util.ArrayList;

@Autonomous(group = "drive")
public class RRVisionTwoConeAutoL extends LinearOpMode {

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

    int scenario = 0;

    AprilTagDetection tagOfInterest = null;

    boolean tagNotDetected = false;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        lift = hardwareMap.get(DcMotorEx.class, "leftShooter");
        claw = hardwareMap.get(Servo.class, "claw");

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


        lift.setTargetPosition(200);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
            boolean tagNotDetected = true;
            int scenario = 0;
           // drive.goingForward(1500, 0.3);
        }

        /* Actually do something useful */
        //if(tagOfInterest == null){
        // leftFront.setPower(0.1);
        //rightFront.setPower(0.1);
        //rightRear.setPower(0.1);
        //leftRear.setPower(0.1);
        //sleep(1000);
        /*
         * Insert your autonomous code here, presumably running some default configuration
         * since the tag was never sighted during INIT
         */
        //}
        //else
        //
        if (tagNotDetected){

        }

        else if(tagOfInterest.id == IDTOI1){
         //   drive.goingLeft(1040,0.5);
            //   drive.goingForward(200,0);
           // drive.goingBackward(700,0.5);
           // drive.goingForward(200,0);
          //  drive.goingForward(1250,0.5);
            int scenario = 1;
        }
        else if(tagOfInterest.id == IDTOI2){
            // drive.goingRight(190, 0.3);
            // drive.goingForward(200, 0);
            // drive.goingBackward(200, 0.3);
            // drive.goingForward(200, 0);
            // drive.goingForward(2000,0.3);
            int scenario = 2;
        }
        else {
            //drive.goingRight(1500,0.5);
            //drive.goingForward(200,0);
            //drive.goingBackward(700,0.5);
            //drive.goingForward(200, 0);
            //drive.goingForward(1150,0.5);
            int scenario = 3;

            /*
            leftFront.setPower(0.2);
            rightFront.setPower(0.2);
            rightRear.setPower(0.2);
            leftRear.setPower(0.2);
            sleep(7000);

             */
        }
        // drive.goingForward(500,0);
        claw.setPosition(0.7);
        /*
         * Insert your autonomous code here, probably using the tag pose to decide your configuration.
         */


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        // while (opModeIsActive()) {sleep(20);}

        Trajectory myTrajectory1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .forward(10)
                .strafeRight(5)
                .forward(5)
                .build();

        Trajectory myTrajectory2 = drive.trajectoryBuilder(new Pose2d())
                .back(5)
                .strafeLeft(36)
                .forward(5)
                .build();

        Trajectory myTrajectory3 = drive.trajectoryBuilder(new Pose2d())
                .back(5)
                .strafeRight(36)
                .forward(5)
                .build();

        Trajectory myTrajectory4 = drive.trajectoryBuilder(new Pose2d())
                .back(5)
                .strafeLeft(5)
                .back(15)
                .build();

        Trajectory myTrajectory5 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(10)
                .forward(10)
                .build();

        Trajectory myTrajectory6 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(20)
                .forward(10)
                .build();

        Trajectory myTrajectory7 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(30)
                .forward(10)
                .build();


        waitForStart();

        if(isStopRequested()) return;

        claw.setPosition(0.9);
        sleep(2000);

        lift.setTargetPosition(5100);
        lift.setPower(1.0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.followTrajectory(myTrajectory1);

        lift.setTargetPosition(1687);
        lift.setPower(0.8);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.followTrajectory(myTrajectory2);

        claw.setPosition(0.9);
        sleep(2000);

        lift.setTargetPosition(5100);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.followTrajectory(myTrajectory3);

        claw.setPosition(0.7);
        sleep(2000);

        lift.setTargetPosition(0);
        lift.setPower(0.8);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.followTrajectory(myTrajectory4);

        if (scenario == 0) {
            drive.followTrajectory(myTrajectory5);
        }

        if (scenario == 1) {
            drive.followTrajectory(myTrajectory6);
        }

        if (scenario == 2){
            drive.followTrajectory(myTrajectory5);
        }

        if (scenario == 3){
            drive.followTrajectory(myTrajectory7);
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

