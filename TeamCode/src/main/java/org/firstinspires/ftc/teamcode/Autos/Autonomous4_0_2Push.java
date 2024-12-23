package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
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


@Config
@Autonomous(name = "A: 4+0")
public class Autonomous4_0_2Push extends LinearOpMode {
    Drivetrain drive = new Drivetrain();
    EnableHand hand = new EnableHand();
    MiggyUnLimbetedOuttake out = new MiggyUnLimbetedOuttake();


    public boolean conditionalEnd(double TARGET) {

        if (out.Rlift.getCurrentPosition() >= TARGET - 25 && out.Rlift.getCurrentPosition() <= TARGET + 25) {
            out.SlidesBrake();

            return false;
        } else {
            return true;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(16.75, -60.75, Math.toRadians(270)));

        hand.init(hardwareMap);
        drive.init(hardwareMap);
        drive.initVisionPortal(hardwareMap);
        out.init(hardwareMap);
//        out.reset();
        out.specimenAutonInit();
        hand.setSwingArmAngle(130);
        hand.open();


        Pose2d highChamberPose = new Pose2d(5, -32, Math.toRadians(270));
        Pose2d highChamberPose2 = new Pose2d(0, -17, Math.toRadians(270));
        Pose2d highChamberPose3 = new Pose2d(2, -13, Math.toRadians(270));
        Pose2d highChamberPose4 = new Pose2d(10, -11, Math.toRadians(270));
        Pose2d sample1Pose = new Pose2d(45, 0, Math.toRadians(270));
        Pose2d sample1PushPose = new Pose2d(50, -40, Math.toRadians(270));
        Pose2d sample2Pose = new Pose2d(57, -3, Math.toRadians(90));
        Pose2d sample2PushPose = new Pose2d(60, -60, Math.toRadians(90));
        Pose2d obsZonePose = new Pose2d(47, -30, Math.toRadians(90));
        Pose2d obsZonePoseforinside = new Pose2d(53, -30, Math.toRadians(90));
        Pose2d obsZonePikcupPoseforinside = new Pose2d(53, -44, Math.toRadians(90));
        Pose2d obsZonePickupPose = new Pose2d(47, -50, Math.toRadians(90));
        Pose2d obsZonePickupPose2 = new Pose2d(47, -48, Math.toRadians(90));
        Pose2d parkPose = new Pose2d(55, -50, Math.toRadians(270));



        Action highChamberTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.toRadians(90)).splineToLinearHeading(highChamberPose, Math.toRadians(90)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, highChamberPose);

        Action lineBack = mecanumDrive.actionBuilder(mecanumDrive.pose).lineToY(-34).build();
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, -34, Math.toRadians(270)));

        Action sample1Traj = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.toRadians(270)).splineToLinearHeading(sample1Pose, Math.toRadians(68)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, sample1Pose);

        Action sample1PushTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.toRadians(270)).splineToLinearHeading(sample1PushPose, Math.toRadians(90)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, sample1PushPose);

        Action sample2Traj = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(90).splineToLinearHeading(sample2Pose, Math.toRadians(0)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, sample2Pose);

        Action sample2PushandPickUpTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).lineToY(-52).build();
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(60, -52, Math.toRadians(90)));

//        Action obsZoneTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(obsZonePose, Math.toRadians(0)).build();
//        mecanumDrive = new MecanumDrive(hardwareMap, obsZonePose);
//
//        Action obsZonePickupTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.toRadians(270)).splineToLinearHeading(obsZonePickupPose, Math.toRadians(270)).build();
//        mecanumDrive = new MecanumDrive(hardwareMap, obsZonePickupPose);

        Action highChamberTraj2 = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.toRadians(180)).splineToLinearHeading(highChamberPose2, Math.toRadians(90)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, highChamberPose2);

        Action obsZoneTraj2 = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(obsZonePose, Math.toRadians(0)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, obsZonePose);

        Action obsZonePickupTraj2 = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.toRadians(270)).splineToLinearHeading(obsZonePickupPose2, Math.toRadians(270)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, obsZonePickupPose2);

        Action highChamberTraj3 = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.toRadians(180)).splineToLinearHeading(highChamberPose3, Math.toRadians(90)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, highChamberPose3);

        Action obsZoneTraj3 = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(obsZonePoseforinside, Math.toRadians(0)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, obsZonePose);

        Action obsZonePickupTraj3 = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.toRadians(270)).splineToLinearHeading(obsZonePikcupPoseforinside, Math.toRadians(270)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, obsZonePikcupPoseforinside);

        Action highChamberTraj4 = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.toRadians(180)).splineToLinearHeading(highChamberPose4, Math.toRadians(90)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, highChamberPose4);
        Action parkTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(parkPose, Math.toRadians(270)).build();

        while (!isStopRequested() && !opModeIsActive()) {
        }

        waitForStart();


        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                highChamberTraj,
                                telemetryPacket -> {
                                    out.PIDLoop(935);
                                    out.outtakeFlipper.setPosition(0.4);
                                    return conditionalEnd(935);
                                }
                        ),

                        new SleepAction(0.2),
                        telemetryPacket -> {
                            out.PIDLoop(500);
                            return conditionalEnd(500);
                        },
                        new SleepAction(0.2),
                        telemetryPacket -> {
                            out.specimenRelease();
                            return false;
                        },
                        new SleepAction(0.1),

                        new ParallelAction(
                                new SequentialAction(lineBack, sample1Traj),

                                telemetryPacket -> {
                                    out.PIDLoop(0);
                                    return conditionalEnd(0);
                                },
                                telemetryPacket -> {
                                    out.specimenPickupStart();
                                    return false;
                                }
                        ),
                        sample1PushTraj,
                        sample2Traj,
                        sample2PushandPickUpTraj,
                        new SleepAction(0.2),
                        new InstantAction(() -> out.specimenPickupGrab()),
//                        telemetryPacket -> {
//                            out.specimenPickupGrab();
//                            return false;
//                        },
                        new SleepAction(0.2),
                        new ParallelAction(
                                highChamberTraj2,
                                new SequentialAction(
                                        telemetryPacket -> {
                                            out.PIDLoop(100);
                                            return conditionalEnd(100);
                                        },
                                        telemetryPacket -> {
                                            out.PIDLoop(935);
                                            out.outtakeFlipper.setPosition(0.4);
                                            return conditionalEnd(935);
                                        }
                                )
                        ),
                        new SleepAction(0.3),
                        telemetryPacket -> {
                            out.PIDLoop(500);
                            return conditionalEnd(500);
                        },
                        new SleepAction(0.2),
                        telemetryPacket -> {
                            out.specimenRelease();
                            return false;
                        },
                        new ParallelAction(obsZoneTraj2,
                               new SequentialAction(
                                telemetryPacket -> {
                                    out.PIDLoop(0);
                                    return conditionalEnd(0);
                                },
                                telemetryPacket -> {
                                    out.specimenPickupStart();
                                    return false;
                                }
                                )
                        ),
                        new SleepAction(0.2),
                        obsZonePickupTraj2,
                        telemetryPacket -> {
                            out.specimenPickupGrab();
                            return false;
                        },
                        new SleepAction(0.2),
                        new ParallelAction(
                                highChamberTraj3,
                                new SequentialAction(
                                        telemetryPacket -> {
                                            out.PIDLoop(100);
                                            return conditionalEnd(100);
                                        },
                                        telemetryPacket -> {
                                            out.PIDLoop(935);
                                            out.outtakeFlipper.setPosition(0.4);
                                            return conditionalEnd(935);
                                        }
                                )
                        ),
                        telemetryPacket -> {
                            out.PIDLoop(500);
                            return conditionalEnd(500);
                        },
                        new SleepAction(0.3),
                        telemetryPacket -> {
                            out.specimenRelease();
                            return false;
                        },
                        new ParallelAction(obsZoneTraj3,
                            new SequentialAction(
                                    telemetryPacket -> {
                                        out.PIDLoop(0);
                                        return conditionalEnd(0);
                                    },
                                    telemetryPacket -> {
                                        out.specimenPickupStart();
                                        return false;
                                    }
                            )
                        ),
                new SleepAction(0.2),
                obsZonePickupTraj3,
                telemetryPacket -> {
                    out.specimenPickupGrab();
                    return false;
                },
                new SleepAction(0.2),
                new ParallelAction(
                        highChamberTraj4,
                        new SequentialAction(
                                telemetryPacket -> {
                                    out.PIDLoop(100);
                                    return conditionalEnd(100);
                                },
                                telemetryPacket -> {
                                    out.PIDLoop(935);
                                    out.outtakeFlipper.setPosition(0.4);
                                    return conditionalEnd(935);
                                }
                        )
                ),
                telemetryPacket -> {
                    out.PIDLoop(500);
                    return conditionalEnd(500);
                },
                new SleepAction(0.3),
                telemetryPacket -> {
                    out.specimenRelease();
                    return false;
                },
        new ParallelAction(
                parkTraj,
                telemetryPacket -> {
                    out.PIDLoop(0);
                    return conditionalEnd(0);
                }
                )
        )
        );

        telemetry.update();


    }
}

