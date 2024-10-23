package org.firstinspires.ftc.teamcode.RoadRunner.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Autonomous (name = "BlueAllianceObsZone")
public class BlueAllianceObsZone extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 40, Math.toRadians(-180)));

        Action myTrajectory = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-35, 52, Math.toRadians(-180)), Math.toRadians(180))

                .splineToLinearHeading(new Pose2d(-48, 42, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-56, 56, Math.toRadians(270)), Math.toRadians(90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-58, 42, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-56, 56, Math.toRadians(270)), Math.toRadians(90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-58, 43, Math.toRadians(-120)), Math.toRadians(-90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-56, 56, Math.toRadians(270)), Math.toRadians(90))
                .waitSeconds(1)
                .turn(Math.toRadians(27))
                .lineToX(-27)
                .build();
        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(myTrajectory));
    }
}
