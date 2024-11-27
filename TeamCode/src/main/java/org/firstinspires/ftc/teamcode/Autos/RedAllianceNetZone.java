package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;


public class RedAllianceNetZone extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-70, 0, Math.toRadians(0)));

        Action myTrajectory = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-35, -52, Math.toRadians(-180)), Math.toRadians(180))
                .turn(Math.toRadians(-25))
                .lineToY(-42)
                .turn(Math.toRadians(-65))
                .waitSeconds(1)
                .turn(Math.toRadians(-10))
                .lineToY(-60)
                .turn(Math.toRadians(-25))
                .waitSeconds(1)
                .turn(Math.toRadians(10))
                .lineToY(-40)
                .turn(Math.toRadians(15))
                .waitSeconds(1)
                .lineToY(-60)
                .turn(Math.toRadians(-30))
                .waitSeconds(1)
                .turn(Math.toRadians(60))
                .lineToY(-40)
                .waitSeconds(1)
                .turn(Math.toRadians(-25))
                .lineToY(-60)
                .build();
        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(myTrajectory));
    }
}

