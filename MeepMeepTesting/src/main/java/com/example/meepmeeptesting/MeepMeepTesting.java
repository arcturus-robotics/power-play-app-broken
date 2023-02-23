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
                .setConstraints(40, 40, Math.toRadians(249.761234517728*0.9), Math.toRadians(184.02607784577722*0.7), 15.76)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(37.6, -64.28125, Math.toRadians(90)))
                                .strafeLeft(2)
                                .forward(50)
                                .turn(Math.toRadians(-38))
                                .turn(Math.toRadians((90+38)))
                                .back(27)
                                .forward(8)
                                .turn(Math.toRadians(54))
                                .turn(Math.toRadians(-54))
                                .back(8)
                                .forward(8)
                                .turn(Math.toRadians(54))
                                .turn(Math.toRadians(-54))
                                .back(8)
                                .forward(8)
                                .turn(Math.toRadians(54))
                                .turn(Math.toRadians(-54))
                                .back(8)
                                .forward(8)
                                .turn(Math.toRadians(54))
                                .turn(Math.toRadians(-54))
                                .back(5)
                                .strafeLeft(21)

                                //for Parking 1, this is fine

                                // for parking 2,add
//                                .forward(24)

                                //for Parking 3,add
//                                 .forward(47)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}