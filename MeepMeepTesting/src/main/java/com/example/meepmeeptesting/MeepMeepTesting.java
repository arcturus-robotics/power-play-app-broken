package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(26.2, 26.2, Math.toRadians(249.761234517728*0.9), Math.toRadians(184.02607784577722*0.7), 15.76)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38, -64.28125, Math.toRadians(90)))
                                .strafeRight(4)
                                .forward(55)
                                .turn(Math.toRadians(-52))
                                .forward(12)
                                .back(11)
                                .turn(Math.toRadians(52))
                                .back(5.2)
                                .turn(Math.toRadians(90))
                                .forward(29)
                                .back(26)
                                .turn(Math.toRadians(42))
                                .forward(10)
                                .back(9)
                                .turn(Math.toRadians(-42))

                                //.turn(Math.toRadians(-45))
                                //.forward(28)
                                //.back(28)
                                //.turn(Math.toRadians(-135))
                                /*
                                .splineTo(new Vector2d(35.42, -36.21), Math.toRadians(90.00))
                                .splineTo(new Vector2d(26.0104, -2.8289), Math.toRadians(138))
                                .back(12)

                                //.turn(Math.toRadians(42))
                                .back(1)
                                .turn(Math.toRadians(-48))
                                .back(20)

                                 */
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}