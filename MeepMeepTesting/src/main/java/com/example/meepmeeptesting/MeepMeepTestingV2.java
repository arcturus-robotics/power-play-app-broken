package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingV2 {

    public static void main(String[] args) {
        double half_bot = 10.375/2;
        double odowidth = 2.75;
        double startx = 24+half_bot;
        double starty = -72+8.5;
        MeepMeep meepMeep = new MeepMeep(600);
        //center of cone 6.5-5.1875 from left chassis
        // center of  cone 13.5 from bottom of chais
        //cone scoring location is 1.3125 on left, 5 in above scoring rotation

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(249.761234517728*0.9), Math.toRadians(184.02607784577722*0.7), 15.76)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(48-half_bot-odowidth, starty, Math.toRadians(90)))
                                /* for robot v2
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

                                 */

                                //.lineToConstantHeading(new Vector2d(36, -48))
//                                .lineTo(new Vector2d(36, -48+27), Math.toRadians(90+45))
                                //.back(27)
                                //.lineToLinearHeading(new Pose2d(24+4.45,0-2.616,Math.toRadians(90+45)))

                                .lineToConstantHeading(new Vector2d(startx+7.5,starty))
                                .lineToConstantHeading(new Vector2d(startx+7.5,starty+50))
                                .lineToConstantHeading(new Vector2d(24+1.3125,starty+50))
                                .lineToConstantHeading(new Vector2d(24+1.3125-0.5513245,-5+0.1))
                                .lineToConstantHeading(new Vector2d(24+1.3125-0.5513245, -14))


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
