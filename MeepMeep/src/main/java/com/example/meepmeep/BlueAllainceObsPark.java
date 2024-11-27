package com.example.meepmeep;

//import static java.lang.VersionProps.build;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueAllainceObsPark {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(550);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -65 , Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(0, -34, Math.toRadians(270)), Math.toRadians(90))
                        .lineToY(-34)
                .splineToLinearHeading(new Pose2d(42, -14, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(46, -49, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(50, -14, Math.toRadians(90)), Math.toRadians(10))
                .splineToLinearHeading(new Pose2d(54, -49, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(55, -14, Math.toRadians(90)), Math.toRadians(10))
                .splineToLinearHeading(new Pose2d(62, -49, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(47, -57, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(0, -34, Math.toRadians(270)), Math.toRadians(90))
                .setTangent(180).splineToLinearHeading(new Pose2d(47, -57, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(0, -34, Math.toRadians(270)), Math.toRadians(90))
                .setTangent(180).splineToLinearHeading(new Pose2d(47, -57, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(0, -34, Math.toRadians(270)), Math.toRadians(90))
                .setTangent(180).splineToLinearHeading(new Pose2d(47, -57, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(47, -59, Math.toRadians(180)), Math.toRadians(270))
                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}