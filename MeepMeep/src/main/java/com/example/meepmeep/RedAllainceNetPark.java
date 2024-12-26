package com.example.meepmeep;

//import static java.lang.VersionProps.build;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedAllainceNetPark {
    public static void main(String[] args) {
        Pose2d highChamberPose = new Pose2d(5, -32, Math.toRadians(270));
        Pose2d highChamberPose2 = new Pose2d(0, -17, Math.toRadians(270));
        Pose2d highChamberPose3 = new Pose2d(2, -15, Math.toRadians(270));
        Pose2d highChamberPose4 = new Pose2d(10, -13, Math.toRadians(270));
        Pose2d sample1Pose = new Pose2d(45, 0, Math.toRadians(270));
        Pose2d sample1PushPose = new Pose2d(50, -40, Math.toRadians(270));
        Pose2d sample2Pose = new Pose2d(57, -3, Math.toRadians(90));
        Pose2d sample2PushPose = new Pose2d(60, -60, Math.toRadians(90));
        Pose2d obsZonePose = new Pose2d(47, -30, Math.toRadians(90));
        Pose2d obsZonePoseforinside = new Pose2d(50, -30, Math.toRadians(90));
        Pose2d obsZonePikcupPoseforinside = new Pose2d(50, -48, Math.toRadians(90));
        Pose2d obsZonePickupPose = new Pose2d(47, -50, Math.toRadians(90));
        Pose2d obsZonePickupPose2 = new Pose2d(47, -48, Math.toRadians(90));
        Pose2d parkPose = new Pose2d(55, -50, Math.toRadians(270));
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(16.75, -60.75, Math.toRadians(270)))
                .setTangent(Math.toRadians(90)).splineToLinearHeading(highChamberPose, Math.toRadians(90))
                .lineToY(-34)
                .setTangent(Math.toRadians(270)).splineToLinearHeading(sample1Pose, Math.toRadians(68))
                .setTangent(Math.toRadians(270)).splineToLinearHeading(sample1PushPose, Math.toRadians(90))
                .setTangent(90).splineToLinearHeading(sample2Pose, Math.toRadians(0))
                        .setTangent(Math.toRadians(270)).splineToLinearHeading(new Pose2d(57, -52, Math.toRadians(90)), Math.toRadians(270))
                .setTangent(Math.toRadians(165)).splineToLinearHeading(highChamberPose2, Math.toRadians(45))
                .splineToLinearHeading(obsZonePose, Math.toRadians(0))
                .setTangent(Math.toRadians(270)).splineToLinearHeading(obsZonePickupPose2, Math.toRadians(270))
                .setTangent(Math.toRadians(165)).splineToLinearHeading(highChamberPose3, Math.toRadians(90))
                .splineToLinearHeading(obsZonePoseforinside, Math.toRadians(0))
                .setTangent(Math.toRadians(270)).splineToLinearHeading(obsZonePikcupPoseforinside, Math.toRadians(270))
                .setTangent(Math.toRadians(180)).splineToLinearHeading(highChamberPose4, Math.toRadians(90))
                .splineToLinearHeading(parkPose, Math.toRadians(270))
                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}