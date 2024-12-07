package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.teamcode.Subsystem.MiggyUnLimbetedOuttake;

@Autonomous(name = "A: Red 1+3 Sample Faster")
public class Red1_3SampleFaster extends LinearOpMode {
    Drivetrain drive = new Drivetrain();
    EnableHand hand = new EnableHand();
    MiggyUnLimbetedOuttake out = new MiggyUnLimbetedOuttake();


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive3 = new MecanumDrive(hardwareMap, new Pose2d(-32.3, -64, Math.toRadians(90)));


        hand.init(hardwareMap);
        drive.init(hardwareMap);
        drive.initVisionPortal(hardwareMap);
        out.init(hardwareMap);
        out.transfer2(); // CLOSES THE CLAW
        hand.setSwingArmAngleAuton(130);
        hand.open();

        Pose2d basketPose = new Pose2d(-56, -53, Math.toRadians(45));
        Pose2d basketPose2 = new Pose2d(-56-1, -53, Math.toRadians(45));
        Pose2d basketPose3 = new Pose2d(-56-2, -53, Math.toRadians(45));
        Pose2d samplePose1 = new Pose2d(-53.6, -44, Math.toRadians(95));
        Pose2d samplePose2 = new Pose2d(-64.5, -43, Math.toRadians(95));
        Pose2d samplePose3 = new Pose2d(-48, -25.5, Math.toRadians(180));

        Action basket1traj = drive3.actionBuilder(drive3.pose).splineToLinearHeading(basketPose, Math.toRadians(280)).build();
        drive3 = new MecanumDrive(hardwareMap, basketPose);

        Action sample1traj = drive3.actionBuilder(drive3.pose).splineToLinearHeading(samplePose1, Math.toRadians(90)).build();
        drive3 = new MecanumDrive(hardwareMap, samplePose1);

        Action basket2traj = drive3.actionBuilder(drive3.pose).splineToLinearHeading(basketPose2, Math.toRadians(280)).build();
        drive3 = new MecanumDrive(hardwareMap, basketPose2);

        Action sample2traj = drive3.actionBuilder(drive3.pose).splineToLinearHeading(samplePose2, Math.toRadians(90)).build();
        drive3 = new MecanumDrive(hardwareMap, samplePose2);

        Action basket3traj = drive3.actionBuilder(drive3.pose).splineToLinearHeading(basketPose3, Math.toRadians(280)).build();
        drive3 = new MecanumDrive(hardwareMap, basketPose3);

        Action sample3traj = drive3.actionBuilder(drive3.pose).splineToLinearHeading(samplePose3, Math.toRadians(90)).build();
        drive3 = new MecanumDrive(hardwareMap, samplePose3);

        Action basket4traj = drive3.actionBuilder(drive3.pose).splineToLinearHeading(basketPose, Math.toRadians(280)).build();

        while (!isStopRequested() && !opModeIsActive()) {
        }
        waitForStart();

        out.autonInit();

        if (isStopRequested()) return;
        Action slidesUpAction = new SequentialAction(
                telemetryPacket -> {
                    out.PIDLoop(1280);
                    telemetry.addData("Slides current pos", out.currentPos);
                    telemetry.update();
                    return false;
                }
        );
        Action slidesDownAction = new SequentialAction(
                telemetryPacket -> {
                    out.PIDLoop(0);
                    telemetry.addData("Slides current pos", out.currentPos);
                    telemetry.update();
                    return false;
                }, new SleepAction(1.0),
                telemetryPacket -> {
                    out.SlidesBrake();
                    telemetry.addLine("brake slides");
                    telemetry.update();
                    return false;
                }

        );
        Action basketActions = new SequentialAction(
                new SleepAction(0.3),
                telemetryPacket -> {
                    telemetry.addLine("Back 1");
                    telemetry.update();
                    out.back1();
                    return false;
                },
                slidesUpAction
                , new SleepAction(0.7),
                telemetryPacket -> {
                    telemetry.addLine("Back");
                    telemetry.update();
                    out.backAuton();
                    return false;
                },
                new SleepAction(0.3),
                telemetryPacket -> {
                    telemetry.addLine("Score");
                    telemetry.update();
                    out.score();
                    return false;
                },
                new SleepAction(0.3),
                telemetryPacket -> {
                    telemetry.addLine("back 1 after placing");
                    telemetry.update();
                    out.back1();
                    return false;
                },
                new SleepAction(0.7),
                slidesDownAction
        );

        Action pickupActions = new SequentialAction(telemetryPacket -> {
            telemetry.addLine("Scan 2");
            telemetry.update();
            hand.scan2();
            return false;
        },
                new SleepAction(0.2),
                telemetryPacket -> {
                    telemetry.addLine("Scan 3");
                    telemetry.update();
                    //hand.scan3();
                    hand.close();
                    return false;
                },
                telemetryPacket -> {
                    telemetry.addLine("Hovering");
                    telemetry.update();
                    hand.hoverAuto();
                    return false;
                },
                new SleepAction(0.1),
                telemetryPacket -> {
                    telemetry.addLine("Picking up 1");
                    telemetry.update();
                    hand.autonPickup1();
                    return false;
                },
                new SleepAction(0.3),
                telemetryPacket -> {
                    telemetry.addLine("Picking up 2");
                    telemetry.update();
                    hand.pickup2Auton();
                    return false;
                },
                new SleepAction(0.4),
                telemetryPacket -> {
                    telemetry.addLine("close claw");
                    telemetry.update();
                    hand.open();
                    return false;
                },
                new SleepAction(0.2),
                telemetryPacket -> {
                    telemetry.addLine("Transfer in one");
                    telemetry.update();
                    hand.intaketrasnferinone();
                    return false;
                },
                new SleepAction(0.5),
                telemetryPacket -> {
                    telemetry.addLine("Transfer 1");
                    telemetry.update();
                    out.transfer1();
                    return false;
                },
                new SleepAction(0.1),
                telemetryPacket -> {
                    telemetry.addLine("Transfer 2");
                    telemetry.update();
                    out.transfer2();
                    return false;
                },
                new SleepAction(0.1),
                telemetryPacket -> {
                    telemetry.addLine("Hand released sample");
                    telemetry.update();
                    hand.close();
                    return false;
                }
        );
        Action slidesUpAction2 = new SequentialAction(
                telemetryPacket -> {
                    out.PIDLoop(1280);
                    telemetry.addData("Slides current pos", out.currentPos);
                    telemetry.update();
                    return false;
                }
        );
        Action slidesDownAction2 = new SequentialAction(
                telemetryPacket -> {
                    out.PIDLoop(0);
                    telemetry.addData("Slides current pos", out.currentPos);
                    telemetry.update();
                    return false;
                }, new SleepAction(1.0),
                telemetryPacket -> {
                    out.SlidesBrake();
                    telemetry.addLine("brake slides");
                    telemetry.update();
                    return false;
                }

        );
        Action basketActions2 = new SequentialAction(
                new SleepAction(0.3),
                telemetryPacket -> {
                    telemetry.addLine("Back 1");
                    telemetry.update();
                    out.back1();
                    return false;
                },
                slidesUpAction2
                , new SleepAction(0.7),
                telemetryPacket -> {
                    telemetry.addLine("Back");
                    telemetry.update();
                    out.backAuton();
                    return false;
                },
                new SleepAction(0.3),
                telemetryPacket -> {
                    telemetry.addLine("Score");
                    telemetry.update();
                    out.score();
                    return false;
                },
                new SleepAction(0.3),
                telemetryPacket -> {
                    telemetry.addLine("back 1 after placing");
                    telemetry.update();
                    out.back1();
                    return false;
                },
                new SleepAction(0.7),
                slidesDownAction2
        );

        Action pickupActions2 = new SequentialAction(telemetryPacket -> {
            telemetry.addLine("Scan 2");
            telemetry.update();
            hand.scan2();
            return false;
        },
                new SleepAction(0.2),
                telemetryPacket -> {
                    telemetry.addLine("Scan 3");
                    telemetry.update();
                    //hand.scan3();
                    hand.close();
                    return false;
                },
                telemetryPacket -> {
                    telemetry.addLine("Hovering");
                    telemetry.update();
                    hand.hoverAuto();
                    return false;
                },
                new SleepAction(0.1),
                telemetryPacket -> {
                    telemetry.addLine("Picking up 1");
                    telemetry.update();
                    hand.autonPickup1();
                    return false;
                },
                new SleepAction(0.3),
                telemetryPacket -> {
                    telemetry.addLine("Picking up 2");
                    telemetry.update();
                    hand.pickup2Auton();
                    return false;
                },
                new SleepAction(0.4),
                telemetryPacket -> {
                    telemetry.addLine("close claw");
                    telemetry.update();
                    hand.open();
                    return false;
                },
                new SleepAction(0.2),
                telemetryPacket -> {
                    telemetry.addLine("Transfer in one");
                    telemetry.update();
                    hand.intaketrasnferinone();
                    return false;
                },
                new SleepAction(0.5),
                telemetryPacket -> {
                    telemetry.addLine("Transfer 1");
                    telemetry.update();
                    out.transfer1();
                    return false;
                },
                new SleepAction(0.1),
                telemetryPacket -> {
                    telemetry.addLine("Transfer 2");
                    telemetry.update();
                    out.transfer2();
                    return false;
                },
                new SleepAction(0.1),
                telemetryPacket -> {
                    telemetry.addLine("Hand released sample");
                    telemetry.update();
                    hand.close();
                    return false;
                }
        );
        Action slidesUpAction3 = new SequentialAction(
                telemetryPacket -> {
                    out.PIDLoop(1280);
                    telemetry.addData("Slides current pos", out.currentPos);
                    telemetry.update();
                    return false;
                }
        );
        Action slidesDownAction3 = new SequentialAction(
                telemetryPacket -> {
                    out.PIDLoop(0);
                    telemetry.addData("Slides current pos", out.currentPos);
                    telemetry.update();
                    return false;
                }, new SleepAction(1.0),
                telemetryPacket -> {
                    out.SlidesBrake();
                    telemetry.addLine("brake slides");
                    telemetry.update();
                    return false;
                }

        );
        Action basketActions3 = new SequentialAction(
                new SleepAction(0.3),
                telemetryPacket -> {
                    telemetry.addLine("Back 1");
                    telemetry.update();
                    out.back1();
                    return false;
                },
                slidesUpAction3
                , new SleepAction(0.7),
                telemetryPacket -> {
                    telemetry.addLine("Back");
                    telemetry.update();
                    out.backAuton();
                    return false;
                },
                new SleepAction(0.3),
                telemetryPacket -> {
                    telemetry.addLine("Score");
                    telemetry.update();
                    out.score();
                    return false;
                },
                new SleepAction(0.3),
                telemetryPacket -> {
                    telemetry.addLine("back 1 after placing");
                    telemetry.update();
                    out.back1();
                    return false;
                },
                new SleepAction(0.7),
                slidesDownAction3
        );

        Action pickupActions3 = new SequentialAction(telemetryPacket -> {
            telemetry.addLine("Scan 2");
            telemetry.update();
            hand.scan2();
            return false;
        },
                new SleepAction(0.2),
                telemetryPacket -> {
                    telemetry.addLine("Scan 3");
                    telemetry.update();
                    //hand.scan3();
                    hand.close();
                    return false;
                },
                telemetryPacket -> {
                    telemetry.addLine("Hovering");
                    telemetry.update();
                    hand.hoverAuto();
                    return false;
                },
                new SleepAction(0.1),
                telemetryPacket -> {
                    telemetry.addLine("Picking up 1");
                    telemetry.update();
                    hand.autonPickup1();
                    return false;
                },
                new SleepAction(0.3),
                telemetryPacket -> {
                    telemetry.addLine("Picking up 2");
                    telemetry.update();
                    hand.pickup2Auton();
                    return false;
                },
                new SleepAction(0.4),
                telemetryPacket -> {
                    telemetry.addLine("close claw");
                    telemetry.update();
                    hand.open();
                    return false;
                },
                new SleepAction(0.2),
                telemetryPacket -> {
                    telemetry.addLine("Transfer in one");
                    telemetry.update();
                    hand.intaketrasnferinone();
                    return false;
                },
                new SleepAction(0.5),
                telemetryPacket -> {
                    telemetry.addLine("Transfer 1");
                    telemetry.update();
                    out.transfer1();
                    return false;
                },
                new SleepAction(0.1),
                telemetryPacket -> {
                    telemetry.addLine("Transfer 2");
                    telemetry.update();
                    out.transfer2();
                    return false;
                },
                new SleepAction(0.1),
                telemetryPacket -> {
                    telemetry.addLine("Hand released sample");
                    telemetry.update();
                    hand.close();
                    return false;
                }
        );
        Action slidesUpAction4 = new SequentialAction(
                telemetryPacket -> {
                    out.PIDLoop(1280);
                    telemetry.addData("Slides current pos", out.currentPos);
                    telemetry.update();
                    return false;
                }
        );
        Action slidesDownAction4 = new SequentialAction(
                telemetryPacket -> {
                    out.PIDLoop(0);
                    telemetry.addData("Slides current pos", out.currentPos);
                    telemetry.update();
                    return false;
                }, new SleepAction(1.0),
                telemetryPacket -> {
                    out.SlidesBrake();
                    telemetry.addLine("brake slides");
                    telemetry.update();
                    return false;
                }

        );

        Action basketActions4 = new SequentialAction(
                new SleepAction(0.3),
                telemetryPacket -> {
                    telemetry.addLine("Back 1");
                    telemetry.update();
                    out.back1();
                    return false;
                },
                slidesUpAction4
                , new SleepAction(0.7),
                telemetryPacket -> {
                    telemetry.addLine("Back");
                    telemetry.update();
                    out.backAuton();
                    return false;
                },
                new SleepAction(0.3),
                telemetryPacket -> {
                    telemetry.addLine("Score");
                    telemetry.update();
                    out.score();
                    return false;
                },
                new SleepAction(0.3),
                telemetryPacket -> {
                    telemetry.addLine("back 1 after placing");
                    telemetry.update();
                    out.back1();
                    return false;
                },
                new SleepAction(0.7),
                slidesDownAction4
        );
        Action basket1Action = new SequentialAction(basket1traj,new SleepAction(0.5), basketActions);
        Action sample1Action = new SequentialAction(sample1traj,new SleepAction(0.5),pickupActions);
        Action basket2Action = new SequentialAction(basket2traj, new SleepAction(0.5), basketActions2);
        Action sample2Action = new SequentialAction(sample2traj, new SleepAction(0.5), pickupActions2);
        Action basket3Action = new SequentialAction(basket3traj, new SleepAction(0.5), basketActions3);
        Action sample3Action = new SequentialAction(sample3traj, new SleepAction(0.5), pickupActions3);
        Action basket4Action = new SequentialAction(basket4traj, new SleepAction(0.5), basketActions4);


        Actions.runBlocking(new SequentialAction(
                basket1Action
                , sample1Action
                , basket2Action
                , sample2Action
                ,basket3Action
                ,sample3Action
                ,basket4Action
        ));
    }
}
