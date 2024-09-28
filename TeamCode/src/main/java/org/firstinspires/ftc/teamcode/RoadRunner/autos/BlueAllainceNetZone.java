package org.firstinspires.ftc.teamcode.RoadRunner.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Autonomous (name = "BlueLlainceNetZone")
public class BlueAllainceNetZone extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(70, 10, Math.toRadians(-110)));

        Action myTrajectory = drive.actionBuilder(drive.pose)
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
                .build();
        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(myTrajectory));
    }
}
