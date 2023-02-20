package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    /*
    LATERAL_DISTANCE: 13.3125 measured, 13.039767 calculated or 13.035990
     */
    public static double LATERAL_DISTANCE = 13.125;// in; distance between the left and right wheels

    /*
       Our robot: Length -> 17" ; Width -> 10 3/8"
       17/2 - 15 9/16 (Measured Position of Middle of Wheel) ->
       Left Odometry Pod Measurement -> 2 3/8" wide
       Right Odometry Pod Measurement -> 2 11/16" wide
     */
    public static double FORWARD_OFFSET = -7.0625; // in; offsedont worry about memorit of the lateral wheel

    /*
        X_MULTIPLIER TEST DATA (TARGET 96 in): 97.987464, 98.091533, 98.060342
        Y_MULTIPLIER TEST DATA (TARGET 96 in): 97.725129, 97.573082, 97.732029

        X_MULTIPLIER DATA: 0.97971716055, 0.97867774173,0.97898903921; AVG: 0.97912798049
        Y_MULTIPLIER DATA: 0.98234712997,0.98387791009,0.98227777507; AVG: 0.98283427171
     */
    //prev 0.979127
    public static double X_MULTIPLIER = 0.988716; // Multiplier in the X direction
    //0.982813
    //40
    //0.994619
    //60
    //All
    //0.988716


    public static double Y_MULTIPLIER = 0.982834; // Multiplier in the Y direction

    private Encoder leftEncoder, rightEncoder, backEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));  

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightShooter"));
        backEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "front"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        //leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(backEncoder.getCurrentPosition() * Y_MULTIPLIER)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(backEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}
