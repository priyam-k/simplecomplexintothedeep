package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;


public class Blue1_6Auton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-46, 0, Math.toRadians(270)));



        // Adjusted trajectory
        Action myTrajectory = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-42, -42, Math.toRadians(0)), Math.toRadians(0))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-56, 56, Math.toRadians(315)), Math.toRadians(170))
                .splineToLinearHeading(new Pose2d(-42, 48, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-56, 56, Math.toRadians(315)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-42, 58, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-56, 56, Math.toRadians(315)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-42, 60, Math.toRadians(20)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-56, 56, Math.toRadians(315)), Math.toRadians(180))
                .turn(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-42, -58, Math.toRadians(0)), Math.toRadians(0))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-56, 56, Math.toRadians(315)), Math.toRadians(180))
                .turn(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-42, -56, Math.toRadians(-45)), Math.toRadians(0))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-56, 56, Math.toRadians(315)), Math.toRadians(180))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(myTrajectory));
    }
}
