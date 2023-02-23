package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveV2;
import org.firstinspires.ftc.teamcode.util.Encoder;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    private Encoder leftEncoder, rightEncoder, backEncoder;
    private DistanceSensor rightDistance;
    private DistanceSensor leftDistance;

    double distl1;
    double distl2;
    double distl3;

    double distr1;
    double distr2;
    double distr3;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveV2 drive = new SampleMecanumDriveV2(hardwareMap);
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightShooter"));
        backEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "front"));

        leftDistance= hardwareMap.get(DistanceSensor.class, "sensor_range_left");
        rightDistance = hardwareMap.get(DistanceSensor.class, "sensor_range_right");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d closeRedTerminal = new Pose2d(-63.5, -64.28125, Math.toRadians(90));
        Pose2d startinglocatiion = new Pose2d(0, 0, Math.toRadians(90));

        drive.setPoseEstimate(startinglocatiion);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            distl1 = leftDistance.getDistance(DistanceUnit.INCH);
            distl2 = leftDistance.getDistance(DistanceUnit.INCH);
            distl3 = leftDistance.getDistance(DistanceUnit.INCH);

            distr1 = rightDistance.getDistance(DistanceUnit.INCH);
            distr2 = rightDistance.getDistance(DistanceUnit.INCH);
            distr3 = rightDistance.getDistance(DistanceUnit.INCH);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("leftencoder", leftEncoder.getCurrentPosition());
            telemetry.addData("rightencoder", rightEncoder.getCurrentPosition());
            telemetry.addData("backencoder", backEncoder.getCurrentPosition());
            telemetry.addData("RIGHTDISTANCE", distr1 + distr2 + distr3/3);
            telemetry.addData("LEFTDISTANCE", distl1+distl2+distl3 / 3);

            telemetry.update();
        }
    }
}
