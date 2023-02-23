package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingV2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(249.761234517728*0.9), Math.toRadians(184.02607784577722*0.7), 15.76)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38, -64.28125, Math.toRadians(90)))
                                .strafeRight(4)
                                .forward(50)
                                .turn(Math.toRadians(-35))
                                .turn(Math.toRadians(-(90-35)))
                                .back(27)
                                .forward(8)
                                .turn(Math.toRadians(-60))
                                .turn(Math.toRadians(60))
                                .back(8)
                                .forward(8)
                                .turn(Math.toRadians(-60))
                                .turn(Math.toRadians(60))
                                .back(8)
                                .forward(8)
                                .turn(Math.toRadians(-60))
                                .turn(Math.toRadians(60))
                                .back(8)
                                .forward(8)
                                .turn(Math.toRadians(-60))
                                .turn(Math.toRadians(60))
                                .back(5)
                                .strafeRight(21)

                                //for Parking 1, this is fine

                                // for parking 2,add
                                 //.forward(22)

                                //for Parking 3,add
//                                 .forward(46)

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
