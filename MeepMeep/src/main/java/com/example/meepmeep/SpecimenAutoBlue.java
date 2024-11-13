package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SpecimenAutoBlue {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(550);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-14, 61, Math.toRadians(90)))

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(0, 35, Math.toRadians(90)), Math.toRadians(270))

                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(-47, 59, Math.toRadians(270)), Math.toRadians(90))

                .waitSeconds(4)
                .splineToLinearHeading(new Pose2d(0, 35, Math.toRadians(90)), Math.toRadians(270))

                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(-47, 59, Math.toRadians(270)), Math.toRadians(90))

                .waitSeconds(4)
                .splineToLinearHeading(new Pose2d(0, 35, Math.toRadians(90)), Math.toRadians(270))

                .setReversed(false)

                .splineToLinearHeading(new Pose2d(-47, 59, Math.toRadians(90)), Math.toRadians(90))
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}