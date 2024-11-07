package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueAllaince {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 34, Math.toRadians(90)))


                .lineToY(45)
                .turn(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(50.6, 39.7, Math.toRadians(-95)), Math.toRadians(-100))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(59.8, 57.5, Math.toRadians(225)), Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(59, 39.7, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(59.8, 57.5, Math.toRadians(225)), Math.toRadians(100))
                .waitSeconds(1)
                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}