package org.firstinspires.ftc.teamcode.drive.opmode.obselete;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//
@Disabled
@Autonomous(group = "drive")
public class RROneConeAuto extends LinearOpMode {

    private DcMotorEx lift;
    private Servo claw;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        lift = hardwareMap.get(DcMotorEx.class, "leftShooter");
        claw = claw = hardwareMap.get(Servo.class, "claw");

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


    }


}

