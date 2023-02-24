package org.firstinspires.ftc.teamcode.drive.opmode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Counter extends OpMode {
    DcMotorEx lift_Right;
    DcMotorEx lift_Left;

    @Override
    public void init() {
        lift_Right = hardwareMap.get(DcMotorEx.class, "rightShooter");

        lift_Left = hardwareMap.get(DcMotorEx.class, "leftShooter");
    }
    @Override
    public void start()  {
        lift_Left.setPower(0);
        lift_Right.setPower(0);
        }

    @Override
    public void loop() {
        lift_Left.setPower(1);
        lift_Right.setPower(1);

    }
}
