package org.firstinspires.ftc.teamcode.drive.opmode.obselete;

// import dandroid.text.style.TabStopSpan;

// import com.acmerobotics.roadrunner.geometry.Pose2d;
// import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.ArcturusDriveNoRR;

import java.util.ArrayList;

/**
 * A teleop for use with a single controller.
 * Of course, it still works with two controllers plugged in,
 * but it will only use one of them.
 */
@TeleOp(group = "drive")
@Disabled

public class TestBed extends OpMode {
    // variables are set
    private ArcturusDriveNoRR drive;
    private DcMotorEx motor1PID, motor2PID, motor3, motor4;
    private Servo servo1, servo2;

    double clawpos;
    double liftpos;

    boolean PID1 = false;
    boolean PID2 = false;
    boolean upsies;



    double MotorMaxSpeed = 0.5;
    int targetpos1 = 0;
    int targetpos2 = 0;

    ArrayList<Boolean> boolArray = new ArrayList<Boolean>();
    int booleanIncrement = 0;

    @Override

    public void init() {
        drive = new ArcturusDriveNoRR(hardwareMap);

        motor1PID = hardwareMap.get(DcMotorEx.class, "motor1PID");
        motor2PID = hardwareMap.get(DcMotorEx.class, "motor2PID");
        motor3 =  hardwareMap.get(DcMotorEx.class, "motor3");
        motor4 = hardwareMap.get(DcMotorEx.class, "motor4");

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
    }

    @Override
    public void start() {
        telemetry.addData("Status", "STARTED!!");
    }


    @Override
    public void loop() {

        double leftFront = -Range.clip(gamepad2.left_stick_y - gamepad2.left_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
        double leftRear = -Range.clip(gamepad2.left_stick_y + gamepad2.right_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
        double rightRear = -Range.clip(gamepad2.right_stick_y - gamepad2.left_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
        double rightFront = -Range.clip(gamepad2.right_stick_y + gamepad2.right_stick_x, -MotorMaxSpeed, MotorMaxSpeed);

        drive.setMotorPowers(leftFront, leftRear, rightFront, rightRear);

        clawpos = servo1.getPosition();
        liftpos = motor1PID.getCurrentPosition();

        if (PID1) {
            if (targetpos1 < 0) {
                targetpos1 = 0;
            }
            motor1PID.setTargetPosition(targetpos1);
            motor1PID.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1PID.setPower(0.5);
            }

        if (PID2) {
            if (targetpos2 < 0) {
                targetpos2 = 0;
            }
            motor2PID.setTargetPosition(targetpos1);
            motor2PID.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2PID.setPower(0.5);
        }


        if (gamepad2.dpad_up) {
            motor3.setPower(0.5);
        } else if (gamepad2.dpad_down) {
            motor3.setPower(-0.5);

        }

        if (gamepad2.right_trigger != 0) {
            targetpos1 += 20;
        }
        else if (gamepad2.left_trigger != 0) {
            targetpos1 -= 20;
        }

        if (gamepad2.right_bumper) {
            targetpos2 += 20;

        } else if (gamepad2.left_bumper) {
            targetpos2 -= 20;
        }

        if (gamepad2.a) {
            servo1.setPosition(Range.clip(clawpos + 0.02, 0, 1));
            // intaketilt.setPosition(0);
        } else if (gamepad2.x) {
            servo1.setPosition(Range.clip(clawpos - 0.02, 0, 1));
            // intaketilt.setPcosition(1);
        }

        if (gamepad2.y) {
            servo2.setPosition(Range.clip(clawpos + 0.02, 0, 1));
        } else if (gamepad2.b) {
            servo2.setPosition(Range.clip(clawpos - 0.02, 0, 1));
        }




        if (gamepad2.share) {
            PID1 = false;
            motor1PID.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


        telemetry.addData("lift stay in place", PID1);
        telemetry.addData("lift pos", liftpos);
        telemetry.addData("claw pos", clawpos);
        telemetry.addData("Steve", 999);

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
