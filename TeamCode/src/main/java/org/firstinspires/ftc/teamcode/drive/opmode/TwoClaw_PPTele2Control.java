// giving the code its package
package org.firstinspires.ftc.teamcode.drive.opmode;

// import dandroid.text.style.TabStopSpan;

// import com.acmerobotics.roadrunner.geometry.Pose2d;
// import com.acmerobotics.roadrunner.geometry.Vector2d;

//importing relevant files
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

// file public class set
@TeleOp
public class TwoClaw_PPTele2Control extends OpMode {
    // variables are set
    
    //drive variable
    private ArcturusDriveNoRR drive;
    
    //motor variables
    private DcMotorEx duckwheel;
    private DcMotorEx lift;
    private DcMotorEx lf;
    private DcMotorEx rf;
    private DcMotorEx rr;
    private DcMotorEx lr;
    
    //Servo variables
    private Servo lclaw, rclaw;
    
    // long variable set
    long delay = 0;

    // position variables set, these set the variables the left claw part, right claw part, and lift respectively
    double lclawpos, rclawpos;
    double liftpos;

    // boolean variables set
    boolean PID = true;
    
    //upsies controls the speed of the motor, if not activated, it will not go as fast
    boolean upsies;
    
    boolean motortoggle = false;
    boolean slowspeed = false;


    /*int high = 7900; for previous spool
    int medium = 5800; for previous spool
    int low = 3550; for previous spool*/
    // 4:7 = circumference of previous spool : circumference of new spool
    
    // int variables set
    
    //max height represents the max height of the lift before it becomes stretched too far
    int maxheight = 4400;
    
    //lift height for suitable scoring on the high pole
    int high = 4400;
    
    //lift height for suitable scoring on the medium pole
    int medium = 3200;
    
    //lift height for suitable scoring on the low pole
    int low = 1950;
    
    //lift height for grabbing cones on the ground
    int ground = 0;
    
    //lift height for suitable scoring on the ground junctions
    int caldera = 750;
    
    //
    int selectedpos = 0;
    
    //this variable represents the working speed of a motor
    double WorkingMotorMax = 0.6825-0.05;
    
    // 0.5 normal
    
    
    double MotorMaxSpeed = 0.8;
    
    //when the claw is facing the back side the left and right names are correct
    //variables for the setting the claw parts to their corresponding positions, lclaw = left claw part, etc.
    double lclaw_open = 0;
    double rclaw_open = 0.4;
    double lclaw_closed = 0.55;
    double rclaw_closed = 0.15;
    
    
    // because targetpos is the targeted position of the lift, we need it on the ground during the beginning of driving
    int targetpos = ground;
    
    //double rightfrontpos,leftfrontpos,leftbackpos,rightbackpos;
    
    
    //boolean array list set
    ArrayList<Boolean> boolArray = new ArrayList<Boolean>();
    int booleanIncrement = 0;

    @Override

    public void init() {
        
        //drive variable set
        drive = new ArcturusDriveNoRR(hardwareMap);

        //  noodle = hardwareMap.get(DcMotorEx.class, "intake");
        
        //duckwheel motor set
        //duckwheel = hardwareMap.get(DcMotorEx.class, "front");
         
        //lift motor set
        lift = hardwareMap.get(DcMotorEx.class, "leftShooter");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setTargetPosition(targetpos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.9);
        //noodle = hardwareMap.get(DcMotorEx.class,"rightShooter");
        
        //left claw part servo set
        lclaw = hardwareMap.get(Servo.class, "lclaw");
        lclaw.setPosition(0);
        
        //right claw part servo set
        rclaw = hardwareMap.get(Servo.class, "rclaw");
        rclaw.setPosition(0);


        //  lf = hardwareMap.get(DcMotorEx.class, "leftFront");
        //rr = hardwareMap.get(DcMotorEx.class, "rightRear");
        //lr = hardwareMap.get(DcMotorEx.class, "leftRear");
        //rf = hardwareMap.get(DcMotorEx.class, "rightFront");


    }

    @Override
    // beginning telementry "printed"
    public void start() {
        telemetry.addData("Status", "STARTED!!");
    }


    @Override
    public void loop() {
        
        // this makes the sticks on our first gamepad control the wheels, imagining the control hub is the back
        double leftFront = -Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
        double leftRear = -Range.clip(gamepad1.left_stick_y + gamepad1.right_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
        double rightRear = -Range.clip(gamepad1.right_stick_y - gamepad1.left_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
        double rightFront = -Range.clip(gamepad1.right_stick_y + gamepad1.right_stick_x, -MotorMaxSpeed, MotorMaxSpeed);

        //        if(gamepad1.right_bumper && motortoggle==false){
//            motortoggle = true;
//        }
           
//        else if(gamepad1.right_bumper && motortoggle==true){
//            motortoggle = false;
//        }
//        else {
//
//        }
        
        // if the motors are being used, then pressing the left bumper on the first game pad will slow the robot down for as long as it is pressed
        if (leftFront != 0 || leftRear != 0 || rightRear != 0 || rightFront != 0) {
            if (gamepad1.left_bumper){
                slowspeed = true;
                drive.setMotorPowers(leftFront*0.4, leftRear*0.4, rightFront*0.4, rightRear*0.4);
            }
//            else if (motortoggle) {
//                slowspeed = true;
//                drive.setMotorPowers(leftFront*0.4, leftRear*0.4, rightFront*0.4, rightRear*0.4);
//            }
            else {
                drive.setMotorPowers(leftFront, leftRear, rightFront, rightRear);
                slowspeed=false;
            }
          // if the motors are being used, then pressing the right or left trigger on the first game pad will make the robot strafe in their corresponding directions, right = right, etc.
        } else {
            if (gamepad1.right_trigger != 0) {
                drive.setMotorPowers(0.7, -0.7, -0.7, 0.7);
            } else if (gamepad1.left_trigger != 0){
                drive.setMotorPowers(-0.7, 0.7, 0.7, -0.7);
            }
             else{
                 drive.setMotorPowers(0, 0, 0,0);
             }
        }
        
        // getting the left claw part and right claw part positions
        lclawpos = lclaw.getPosition();
        rclawpos = rclaw.getPosition();
        
        // getting the lift part positions
        liftpos = lift.getCurrentPosition();

        // claw set and lifted to a position
        if (PID) {
            if (targetpos > maxheight) {
                targetpos = maxheight;
            }
            if (targetpos < 0) {
                targetpos = 0;
            }
            lift.setTargetPosition(targetpos);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (upsies) {
                lift.setPower(0.9);
            } else {
                lift.setPower(0.9);
            }
        }

        //planning for a delay
        if (delay != 0) {
            if (System.nanoTime() > 2e+8 + delay) {
                delay = 0;
                PID=true;
                targetpos=selectedpos;
            }
        }
        
        // if the up button on the dpad on the second gamepad is pressed, the lift moves up without an encoder as long as it is pressed
        if (gamepad2.dpad_up) {
            PID = false;
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setPower(0.9);

            //  noodle.setPower(-1);
            
        // if the down button on the dpad on the second gamepad is pressed, the lift moves down without an encoder as long as it is pressed
        } else if (gamepad2.dpad_down) {
            PID = false;
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setPower(-1);

            //noodle.setPower(1);
            
            //if the lift PID is not activated and the no button on the second controller's dpad is activated, then the lift will stay put without an encoder
        } else if (!PID) {
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setPower(0);
            //          lift.setTargetPosition(0);

        }
       // if the second game pad's right trigger is activated, then the lift will go down by 20 units
        if (gamepad2.right_trigger != 0) {
            targetpos -= 20;
            upsies = true;
       // if the second game pad's left trigger is activated, then the lift will go up by 20 units
        } else if (gamepad2.left_trigger != 0) {
            targetpos += 20;
            upsies = true;
        }


        //the following 5 if/else if statements tell the robot which stage of the lift
        //uses the right of the second controller with y referring to up, x referring to medium, a referring to down, b referring to default
        //the left button on the d_pad controller sets the lift to a suitable position for scoring on ground junctions
        //the preference goes y,x,a, and b from most powerful command to least powerful command
        
        if (gamepad2.a) {
            selectedpos = low;
            upsies = true;
            delay=System.nanoTime();
            lclaw.setPosition(lclaw_closed);
            rclaw.setPosition(rclaw_closed);
        } else if (gamepad2.x) {
            selectedpos=medium;
            upsies = true;
            delay=System.nanoTime();
            lclaw.setPosition(lclaw_closed);
            rclaw.setPosition(rclaw_closed);
        } else if (gamepad2.y) {
            selectedpos=high;
            upsies = true;
            delay=System.nanoTime();
            lclaw.setPosition(lclaw_closed);
            rclaw.setPosition(rclaw_closed);
        } else if (gamepad2.b) {
            upsies = false;
            //claw.setPosition(1);
            targetpos = ground;
            PID = true;

        } else if (gamepad2.dpad_left){
            upsies = false;
            targetpos = caldera;
            PID = true;
        }

      //gamepad 2's right bumper makes the claw more closed
      //gamepad 2's left bumper makes the claw more open the claw
        if (gamepad2.right_bumper) {
            /*
            lclaw.setPosition(Range.clip(lclawpos+ 0.01, 0, 1));
            rclaw.setPosition(Range.clip(rclawpos+ 0.01, 0, 1));
             */
            lclaw.setPosition(lclaw_closed);
            rclaw.setPosition(rclaw_closed);
        } else if (gamepad2.left_bumper) {
            /*
            lclaw.setPosition(Range.clip(lclawpos - 0.01, 0, 1));
            rclaw.setPosition(Range.clip(rclawpos - 0.01, 0, 1));
             */
            lclaw.setPosition(lclaw_open);
            rclaw.setPosition(rclaw_open);
        }
        /*
        else {
            //intaketilt.setPower(0);5
        }*/
        
        //planning for all scenarios
        if (gamepad2.share) {
            PID = false;
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

         
        // telemetry.addData("duckspeed",duckspeed);
        
        //telementry added
        telemetry.addData("lift stay in place", PID);
        telemetry.addData("lift pos", liftpos);
        telemetry.addData("left claw pos", lclawpos);
        telemetry.addData("right claw pos", rclawpos);
//        telemetry.addData("Steve", 999);
//        telemetry.addData("Is toggled?", motortoggle);
        telemetry.addData("Is slow?", slowspeed);

        // telemetry.addData("right front", rightfrontpos);
        //telemetry.addData("right rear", rightbackpos);
        //telemetry.addData("left front", leftfrontpos);
        //telemetry.addData("left rear", leftbackpos );
        telemetry.update();
         
        
        //boolean Incrememnt set
        booleanIncrement = 0;


    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopped..");
    }
    
    
    //boolean Array functions
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
