package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedAllaince {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(550);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -36, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(0, -45, Math.toRadians(175)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-51.8, -43, Math.toRadians(85)), Math.toRadians(90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-61, -47.5, Math.toRadians(40)), Math.toRadians(280))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-69, -43, Math.toRadians(85)), Math.toRadians(90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-61, -47.5, Math.toRadians(40)), Math.toRadians(280))
                .waitSeconds(1)
                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}