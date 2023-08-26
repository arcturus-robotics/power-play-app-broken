package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ArcturusDriveNoRR_OG {

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private DcMotorEx leftRear;


    public ArcturusDriveNoRR_OG(HardwareMap hardwareMap)    {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");


    }

    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void setMotorPowers(double lf, double lr, double rf, double rr){
        leftFront.setPower(lf);
        leftRear.setPower(lr);
        rightFront.setPower(rf);
        rightRear.setPower(rr);
    }

    public void goingLeft (long time, double speed){
        //drive.setMotorPowers(0.7, -0.7, 0.7, -0.7);
        setMotorPowers(speed, -speed, speed, -speed);
        sleep(time);
        setMotorPowers(0,0,0,0);
    }

    public void goingRight (long time, double speed){
        //drive.setMotorPowers(-0.7, 0.7, -0.7, 0.7);
        setMotorPowers(-speed, speed, -speed, speed);
        sleep(time);
        setMotorPowers(0,0,0,0);
    }

    public void goingForward(long time, double speed) {
        setMotorPowers(speed, speed, speed, speed);
        sleep(time);
        setMotorPowers(0, 0, 0, 0);

    }

    public void goingBackward (long time, double speed) {
        setMotorPowers(-speed, -speed, -speed, -speed);
        sleep(time);
        setMotorPowers(0, 0, 0, 0);
    }
    public void turnLeft (long time, double speed) {
        setMotorPowers(-speed, -speed, speed, speed);
        sleep(time);
        setMotorPowers(0,0,0,0);
    }
    public void turnRight (long time, double speed) {
        setMotorPowers(speed, speed, -speed, -speed);
        sleep(time);
        setMotorPowers(0,0,0,0);
    }
}
