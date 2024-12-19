package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
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
import org.opencv.core.Mat;


@Config
@Autonomous(name = "A: Auto push 2+0")
public class Autonomous2_0_1Push extends LinearOpMode {
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
        out.reset();


        out.specimenAutonInit();
        hand.setSwingArmAngle(130);
        hand.open();


        Pose2d highChamberPose = new Pose2d(0, -31, Math.toRadians(270));
        Pose2d highChamberPose2 = new Pose2d(-3, -17, Math.toRadians(270));
        Pose2d sample1Pose = new Pose2d(47, 0, Math.toRadians(270));
        Pose2d sample1PushPose = new Pose2d(50, -40, Math.toRadians(270));
        Pose2d obsZonePose = new Pose2d(47, -30, Math.toRadians(90));
        Pose2d obsZonePickupPose = new Pose2d(47, -50, Math.toRadians(90));
        Pose2d obsZonePickupPose2 = new Pose2d(47, -48, Math.toRadians(90));
        Pose2d highChamberPose3 = new Pose2d(-5, -15, Math.toRadians(270));
        Pose2d parkPose = new Pose2d(47, -61, Math.toRadians(270));



        Action highChamberTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.toRadians(90)).splineToLinearHeading(highChamberPose, Math.toRadians(90)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, highChamberPose);

        Action lineBack = mecanumDrive.actionBuilder(mecanumDrive.pose).lineToY(-34).build();
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, -34, Math.toRadians(270)));

        Action sample1Traj = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.toRadians(270)).splineToLinearHeading(sample1Pose, Math.toRadians(68)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, sample1Pose);

        Action sample1PushTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.toRadians(270)).splineToLinearHeading(sample1PushPose, Math.toRadians(90)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, sample1PushPose);



        Action obsZoneTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(obsZonePose, Math.toRadians(0)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, obsZonePose);

        Action obsZonePickupTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.toRadians(270)).splineToLinearHeading(obsZonePickupPose, Math.toRadians(270)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, obsZonePickupPose);

        Action highChamberTraj2 = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.toRadians(90)).splineToLinearHeading(highChamberPose2, Math.toRadians(90)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, highChamberPose2);

        Action obsZoneTraj2 = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(obsZonePose, Math.toRadians(0)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, obsZonePose);

        Action obsZonePickupTraj2 = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.toRadians(270)).splineToLinearHeading(obsZonePickupPose2, Math.toRadians(270)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, obsZonePickupPose);
        Action highChamberTraj3 = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.toRadians(90)).splineToLinearHeading(highChamberPose2, Math.toRadians(90)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, highChamberPose3);

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
                                    out.PIDLoop(900);
                                    out.outtakeFlipper.setPosition(0.4);
                                    return conditionalEnd(900);
                                }
                        ),

                        new SleepAction(0.5),
                        telemetryPacket -> {
                            out.PIDLoop(500);
                            return conditionalEnd(500);
                        },
                        new SleepAction(0.3),
                        telemetryPacket -> {
                            out.specimenRelease();
                            return false;
                        },
                        new SleepAction(0.3),

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
                        obsZoneTraj,
                        new SleepAction(0.5),
                        obsZonePickupTraj,

                        new SleepAction(0.5),
                        telemetryPacket -> {
                            out.specimenPickupGrab();
                            return false;
                        },

                        new SleepAction(0.5),

                        new ParallelAction(
                                highChamberTraj2,
                                new SequentialAction(
                                        telemetryPacket -> {
                                            out.PIDLoop(100);
                                            return conditionalEnd(100);
                                        },
                                        new SleepAction(0.4),
                                        telemetryPacket -> {
                                            out.PIDLoop(935);
                                            out.outtakeFlipper.setPosition(0.4);
                                            return conditionalEnd(935);
                                        }
                                )
                        ),
                        new SleepAction(0.7),
                        telemetryPacket -> {
                            out.PIDLoop(500);
                            return conditionalEnd(500);
                        },
                        new SleepAction(0.3),
                        telemetryPacket -> {
                            out.specimenRelease();
                            return false;
                        },
                        new SleepAction(0.5),
                        new ParallelAction(
                                obsZoneTraj,
                                telemetryPacket -> {
                                    out.PIDLoop(0);
                                    return conditionalEnd(0);
                                }
                        ),
                        new ParallelAction(obsZoneTraj2,
                                telemetryPacket -> {
                            out.specimenPickupStart();
                            return false;
                        }),
        new SleepAction(0.5),
                obsZonePickupTraj2,

                new SleepAction(0.5),
                telemetryPacket -> {
                    out.specimenPickupGrab();
                    return false;
                },

                new SleepAction(0.5),

                new ParallelAction(
                        highChamberTraj3,
                        new SequentialAction(
                                telemetryPacket -> {
                                    out.PIDLoop(100);
                                    return conditionalEnd(100);
                                },
                                new SleepAction(0.4),
                                telemetryPacket -> {
                                    out.PIDLoop(935);
                                    out.outtakeFlipper.setPosition(0.4);
                                    return conditionalEnd(935);
                                }
                        )
                ), telemetryPacket -> {
                            out.PIDLoop(0);
                            out.outtakeFlipper.setPosition(0.4);
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
                                })

            )
        );

        telemetry.update();


    }
}

