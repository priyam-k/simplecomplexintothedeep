package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TrajectoryTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 60, Math.toRadians(90)));

        Action myTrajectory = drive.actionBuilder(drive.pose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3)
                .build();
        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(myTrajectory));
    }

    //                        .lineToY(24)
//                        .waitSeconds(2.0)
//                        .lineToY(40)
//                        .turn(Math.toRadians(-90))
//                        .lineToX(-48)
//                        .turn(Math.toRadians(90))
//                        .lineToY(33)
//                        .waitSeconds(1.0)
//                        .lineToY(40)
//                        .turn(Math.toRadians(90))
//                        .lineToX(55)
//                        .turn(Math.toRadians(-90))
//                        .lineToY(63)
//                        .waitSeconds(1.0)
//                        .lineToY(40)
//                        .turn(Math.toRadians(-90))
//                        .lineToX(-58)
//                        .turn(Math.toRadians(90))
//                        .lineToY(33)
//                        .waitSeconds(1.0)
//                        .lineToY(40)
//                        .turn(Math.toRadians(90))
//                        .lineToX(55)
//                        .turn(Math.toRadians(-90))
//                        .lineToY(63)
//                        .waitSeconds(1.0)
//                        .lineToY(40)
//                        .turn(Math.toRadians(-90))
//                        .lineToX(-68)
//                        .turn(Math.toRadians(90))
//                        .lineToY(33)
//                        .waitSeconds(1.0)
//                        .lineToY(40)
//                        .turn(Math.toRadians(90))
//                        .lineToX(55)
//                        .turn(Math.toRadians(-90))
//                        .lineToY(63)
    }

