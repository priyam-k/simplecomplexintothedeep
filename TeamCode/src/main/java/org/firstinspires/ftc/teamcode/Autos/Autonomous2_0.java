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
@Autonomous(name = "A: Auto 2+0")
public class Autonomous2_0 extends LinearOpMode {
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


        out.transfer2();
        out.specimenAutonInit();
        hand.setSwingArmAngleAuton(130);
        hand.open();


        Pose2d highChamberPose = new Pose2d(0, -33, Math.toRadians(270));
        Pose2d highChamberPose2 = new Pose2d(-4, -33.5, Math.toRadians(270));
        Pose2d obsZonePose = new Pose2d(47, -52, Math.toRadians(90));
        Pose2d parkPose = new Pose2d(47, -61, Math.toRadians(270));
        Pose2d obsZonePickupPose = new Pose2d(47, -64, Math.toRadians(90));

        Action highChamberTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.toRadians(90)).splineToLinearHeading(highChamberPose, Math.toRadians(90)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, highChamberPose);

        Action lineBack = mecanumDrive.actionBuilder(mecanumDrive.pose).lineToY(-34).build();
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, -34, Math.toRadians(270)));

        Action obsZoneTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(obsZonePose, Math.toRadians(0)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, obsZonePose);

        Action obsZonePickupTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.toRadians(270)).splineToLinearHeading(obsZonePickupPose, Math.toRadians(270)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, obsZonePickupPose);

        Action highChamberTraj2 = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.toRadians(90)).splineToLinearHeading(highChamberPose2, Math.toRadians(90)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, highChamberPose2);

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
                                new SequentialAction(lineBack, obsZoneTraj),

                                telemetryPacket -> {
                                    out.PIDLoop(0);
                                    return conditionalEnd(0);
                                },
                                telemetryPacket -> {
                                    out.specimenPickupStart();
                                    return false;
                                }
                        ),

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
                                        telemetryPacket ->{
                                            out.PIDLoop(100);
                                            return conditionalEnd(100);
                                        },
                                        new SleepAction(0.4),
                                        telemetryPacket ->{
                                            out.PIDLoop(900);
                                            out.outtakeFlipper.setPosition(0.4);
                                            return conditionalEnd(900);
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
