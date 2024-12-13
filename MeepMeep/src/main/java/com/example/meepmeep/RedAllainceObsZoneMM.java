package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedAllainceObsZoneMM {
    public static void main(String[] args) {
        Pose2d highChamberPose = new Pose2d(0, -33, Math.toRadians(270));
        Pose2d highChamberPose2 = new Pose2d(2, -33.5, Math.toRadians(270));
        Pose2d obsZonePose = new Pose2d(47, -52, Math.toRadians(90));
        Pose2d parkPose = new Pose2d(47, -61, Math.toRadians(270));
        Pose2d obsZonePickupPose = new Pose2d(47, -64, Math.toRadians(90));
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -62, Math.toRadians(270)))

                .splineToLinearHeading(highChamberPose, Math.toRadians(90))
                        .lineToY(-34)
                        .splineToLinearHeading(obsZonePose, Math.toRadians(0))
                        .splineToLinearHeading(obsZonePickupPose, Math.toRadians(270))
                .setTangent(Math.toRadians(90)).splineToLinearHeading(highChamberPose2, Math.toRadians(90))
                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}