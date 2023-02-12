package org.firstinspires.ftc.teamcode.drive.opmode;

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

import org.firstinspires.ftc.teamcode.drive.ArcturusDriveNoRR;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.ArrayList;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class PPVisionAutoL extends LinearOpMode
{
    private ArcturusDriveNoRR drive;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    private DcMotorEx lift,leftFront, leftRear, rightRear, rightFront;
    private Servo claw;
    private TouchSensor touch_sensor;

    private DistanceSensor sensorRange_left;
    private DistanceSensor sensorRange_right;


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

    @Override
    public void runOpMode()
    {
        //
        // intaketilt = hardwareMap.get(Servo.class, "ringpusher");
        drive = new ArcturusDriveNoRR(hardwareMap);
        lift =  hardwareMap.get(DcMotorEx.class, "leftShooter");
        lift.setTargetPosition(0);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(0.9);

        touch_sensor = hardwareMap.get(TouchSensor.class, "touch");

        sensorRange_left = hardwareMap.get(DistanceSensor.class, "sensor_range_left");
        sensorRange_right = hardwareMap.get(DistanceSensor.class, "sensor_range_right");

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

//        lift.setTargetPosition(200);
//        lift.setPower(1);
//        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
            drive.goingRight(300,0.5);
            drive.goingForward(200,0);
            drive.goingForward(2000,0.3);
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
            drive.goingLeft(1100,0.5);
            drive.goingForward(400,0);
            drive.goingBackward(400,0.5);
            drive.goingForward(400, 0);
            drive.goingForward(1700,0.3);
        }
        else if(tagOfInterest.id == IDTOI2){
            drive.goingRight(125,0.5);
            drive.goingForward(400,0);
            drive.goingBackward(400,0.5);
            drive.goingForward(400, 0);
            drive.goingForward(2000,0.3);
            }
        else {
            drive.goingRight(1450,0.5);
            drive.goingForward(400,0);
            drive.goingBackward(400,0.5);
            drive.goingForward(400, 0);
            drive.goingForward(2000,0.3);

            /*
            leftFront.setPower(0.2);
            rightFront.setPower(0.2);
            rightRear.setPower(0.2);
            leftRear.setPower(0.2);
            sleep(7000);

             */
        }
        drive.goingForward(500,0);
        claw.setPosition(0.7);
        lift.setTargetPosition(0);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        /*
         * Insert your autonomous code here, probably using the tag pose to decide your configuration.
         */


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        // while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("touching", touch_sensor.isPressed()));
        telemetry.addLine(String.format("left distance", sensorRange_left.getDistance(DistanceUnit.INCH)));
        telemetry.addLine(String.format("right distance", sensorRange_right.getDistance(DistanceUnit.INCH)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}