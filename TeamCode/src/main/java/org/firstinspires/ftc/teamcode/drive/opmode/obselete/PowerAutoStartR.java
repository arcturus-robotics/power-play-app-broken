package org.firstinspires.ftc.teamcode.drive.opmode.obselete;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.ArcturusDriveRR;

@Disabled
@Autonomous(group = "drive")
public class PowerAutoStartR extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    private static final Pose2d ORIGIN = new Pose2d(-63.0, -56.0, 0.0);

    private ArcturusDriveRR drive;
    private double number;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private Servo claw;
    double WorkingMotorMax = 0.6825;
    private long freightboxdelay;
    // private ArcturusVision vision;

    @Override
    public void runOpMode() {
        number = 0.5;
        // Dri
        drive = new ArcturusDriveRR(hardwareMap);
        //
        // intaketilt = hardwareMap.get(Servo.class, "ringpusher");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(0.9);

        //leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("sdjflkasfjsdlaf", "press play or you have 5 seconds to live");
        telemetry.update();
        waitForStart();


        //intaketilt.setPosition(0.58);

        //going up to the Lego
        drive.setMotorPowers(-0.3,0.3,-0.3/WorkingMotorMax,0.3);
        sleep(3000);
        //should be at the lego know
        }


    }

