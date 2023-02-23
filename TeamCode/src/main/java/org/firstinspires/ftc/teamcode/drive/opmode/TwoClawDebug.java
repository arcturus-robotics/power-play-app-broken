package org.firstinspires.ftc.teamcode.drive.opmode;

// import dandroid.text.style.TabStopSpan;
// import com.acmerobotics.roadrunner.geometry.Pose2d;
// import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

/**
 * A teleop for use with a single controller.
 * Of course, it still works with two controllers plugged in,
 * but it will only use one of them.
 */
@TeleOp
public class TwoClawDebug extends OpMode {
    //private ArcturusDriveNoRR drive;
    private DcMotorEx lift_Right, lift_Left, horizontal_slide;
    private Servo lclaw, rclaw;
    //private Touch touch;

    long liftdelay = 0;
    long horizdelay = 0;

    double lclawpos, rclawpos;
    double liftpos_right, liftpos_left, horizpos;

    boolean liftPID = true;
    boolean horizPID = true;
    boolean drive_slowspeed = false;

    /*int high = 7900; for previous spool
    int medium = 5800; for previous spool
    int low = 3550; for previous spool*/
    // 4:7 = circumference of previous spool : circumference of new spool
    int maxheight = 4400;
    int highj = 4400;
    int medj = 3200;
    int lowj = 1850;
    int nodej = 750;
    int ground = 0;
    int liftSelectedPos = 0;
    int liftTargetPos = ground;

    int maxhoriz = 2000;
    int scoringPos = 1503;
    int intakePos = 0;
    int horizSelectedPos = 0;
    int horizTargetPos = intakePos;

    double lclaw_open = 0.35;
    double rclaw_open = 0.25;
    double lclaw_closed = 0.5+0.045;
    double rclaw_closed = 0.15-0.045;

    //double WorkingMotorMax = 0.6825-0.05;
    double MotorMaxSpeed = 0.8;

    ArrayList<Boolean> boolArray = new ArrayList<Boolean>();
    int booleanIncrement = 0;

    @Override

    public void init() {
        ///drive = new ArcturusDriveNoRR(hardwareMap);

        lclaw = hardwareMap.get(Servo.class, "lclaw");
        rclaw = hardwareMap.get(Servo.class, "rclaw");
        lclaw.setPosition(lclaw_closed);
        rclaw.setPosition(rclaw_closed);

        // touch = hardwareMap.get(TouchSensor.class, "Touch");

    }

    @Override
    public void start() {
        telemetry.addData("Status", "STARTED!!");
    }


    @Override
    public void loop() {

        if (gamepad2.right_bumper) {
            lclaw.setPosition(Range.clip(lclawpos+ 0.01, 0, 1));
            rclaw.setPosition(Range.clip(rclawpos+ 0.01, 0, 1));
        } else if (gamepad2.left_bumper) {
            lclaw.setPosition(Range.clip(lclawpos - 0.01, 0, 1));
            rclaw.setPosition(Range.clip(rclawpos - 0.01, 0, 1));
        }

        lclawpos = lclaw.getPosition();
        rclawpos = rclaw.getPosition();

        telemetry.addData("lclaw pos", lclawpos);
        telemetry.addData("rclaw pos", rclawpos);
        telemetry.update();

        booleanIncrement = 0;
    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopped..");
    }

    private boolean ifpressed(boolean button) {
        boolean output = false;
        if (boolArray.size() == booleanIncrement) {
            boolArray.add(false);
        }
        boolean buttonWas = boolArray.get(booleanIncrement);
        if (button != buttonWas && button == true) {
            output = true;
        }
        boolArray.set(booleanIncrement, button);
        booleanIncrement += 1;
        return output;
    }

}