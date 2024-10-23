package org.firstinspires.ftc.teamcode.RoadRunner.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Autonomous (name = "RedAllianceObsZone")
public class RedAllianceObsZone extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(70, -10, Math.toRadians(110)));

        Action myTrajectory = drive.actionBuilder(drive.pose)
                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(-42)
                .turn(Math.toRadians(88))
                .lineToX(60)
                .turn(Math.toRadians(92))
                .waitSeconds(3)
                .turn(Math.toRadians(30))
                .waitSeconds(1)
                .turn(Math.toRadians(-30))
                .waitSeconds(1)
                .turn(Math.toRadians(-40))
                .waitSeconds(3)
                .turn(Math.toRadians(40))
                .waitSeconds(2)
                .build();
        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(myTrajectory));
    }
}
