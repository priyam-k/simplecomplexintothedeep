package org.firstinspires.ftc.teamcode.RoadRunner.autos;

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
@Autonomous(name = "Blue 1+6 Auton")
public class Blue1_6Auton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 46, Math.toRadians(-180)));


        Action myTrajectory = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-48, 42, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(225)), Math.toRadians(80))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(48, 42, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(225)), Math.toRadians(90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(58, 42, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(225)), Math.toRadians(90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(65, 42, Math.toRadians(-70)), Math.toRadians(-90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(225)), Math.toRadians(90))
                .waitSeconds(1)
                .turn(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-58, 42, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(225)), Math.toRadians(90))
                .waitSeconds(1)
                .turn(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-53, 42, Math.toRadians(-135)), Math.toRadians(-90))
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(225)), Math.toRadians(90))
                .waitSeconds(1)
                .build();


        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(myTrajectory));

    }
}