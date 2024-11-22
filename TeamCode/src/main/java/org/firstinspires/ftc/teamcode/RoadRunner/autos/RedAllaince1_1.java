package org.firstinspires.ftc.teamcode.RoadRunner.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;


public class RedAllaince1_1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(16.75, -60.75, Math.toRadians(270)));

        Action myTrajectory = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(0, -35, Math.toRadians(270)), Math.toRadians(90))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(47, -59, Math.toRadians(90)), Math.toRadians(270))
                .waitSeconds(4)
                .splineToLinearHeading(new Pose2d(0, -35, Math.toRadians(270)), Math.toRadians(90))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(47, -59, Math.toRadians(90)), Math.toRadians(270))
                .waitSeconds(4)
                .splineToLinearHeading(new Pose2d(0, -35, Math.toRadians(270)), Math.toRadians(90))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(47, -59, Math.toRadians(270)), Math.toRadians(270))
                .build();
        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(myTrajectory));
    }
}
