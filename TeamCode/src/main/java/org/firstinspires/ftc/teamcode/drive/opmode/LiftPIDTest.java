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
import org.firstinspires.ftc.teamcode.drive.ArcturusDriveNoRRViper;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



import java.util.ArrayList;
@TeleOp
public class LiftPIDTest extends OpMode {
    private DcMotorEx lift_Right, lift_Left;
    int liftpos = 0;

    @Override
    public void init() {
        lift_Right = hardwareMap.get(DcMotorEx.class, "rightShooter");
        lift_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_Right.setTargetPosition(liftpos);
        lift_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift_Right.setPower(0.5);

        lift_Left= hardwareMap.get(DcMotorEx.class, "leftShooter");
        lift_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_Left.setTargetPosition(liftpos);
        lift_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift_Left.setPower(0.5);

        lift_Left.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void start() {
        telemetry.addData("Status", "STARTED!!");
    }

    @Override
    public void loop() {

        lift_Left.setTargetPosition(liftpos);
        lift_Right.setTargetPosition(liftpos);

        if (gamepad2.right_trigger != 0) {
            liftpos -= 1;
        } else if (gamepad2.left_trigger != 0) {
            liftpos += 1;
        }

        if (liftpos < 0) {
            liftpos = 0;
        }

        telemetry.addData("left lift pos", lift_Left.getCurrentPosition());
        telemetry.addData("right lift pos", lift_Right.getCurrentPosition());
        telemetry.addData("lift target pos", liftpos);
        telemetry.update();

    }
}
