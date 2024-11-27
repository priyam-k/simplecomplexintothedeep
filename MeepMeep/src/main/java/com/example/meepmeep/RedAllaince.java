package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedAllaince {


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(550);
        Pose2d basketPose = new Pose2d(-57, -57, Math.toRadians(45));
        Pose2d samplePose1 = new Pose2d(-52, -48, Math.toRadians(90));
        Pose2d samplePose2 = new Pose2d(-65.5, -50, Math.toRadians(90));
        Pose2d samplePose3 = new Pose2d(-47, -25.5, Math.toRadians(180));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-30.75, -63.25, Math.toRadians(90)))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(basketPose, Math.toRadians(280))
                .splineToLinearHeading(samplePose1, Math.toRadians(90))
                .splineToLinearHeading(basketPose, Math.toRadians(280))
                .splineToLinearHeading(samplePose2, Math.toRadians(90))
                .splineToLinearHeading(basketPose, Math.toRadians(280))
                .splineToLinearHeading(samplePose3, Math.toRadians(90))
                .splineToLinearHeading(basketPose, Math.toRadians(280))
                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}