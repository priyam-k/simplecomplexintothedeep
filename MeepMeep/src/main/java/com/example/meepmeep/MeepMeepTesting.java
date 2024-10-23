package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(70, 10, -110))
                        .lineToX(30)
                        .turn(Math.toRadians(-85))
                        .lineToY(43)
                        .turn(Math.toRadians(-95))
                        .lineToX(56)
                        .turn(Math.toRadians(-80))
                        .lineToY(56)
                        .turn(Math.toRadians(-50))
                        .lineToY(43)
                        .turn(Math.toRadians(60))
                        .waitSeconds(1)
                        .turn(Math.toRadians(-60))
                        .lineToY(56)
                        .waitSeconds(1)
                        .turn(Math.toRadians(70))
                        .lineToY(43)
                        .waitSeconds(1)
                        .lineToY(56)
                        .turn(Math.toRadians(-70))

                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}