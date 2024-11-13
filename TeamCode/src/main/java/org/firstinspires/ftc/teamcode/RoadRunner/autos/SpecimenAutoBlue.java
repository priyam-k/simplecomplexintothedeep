package org.firstinspires.ftc.teamcode.RoadRunner.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.teamcode.Subsystem.MiggyUnLimbetedOuttake;

@Autonomous(name = "A blue specimen")
public class SpecimenAutoBlue extends LinearOpMode {


    EnableHand hand;
    MiggyUnLimbetedOuttake out;
    DcMotorEx slideMotorRight, slideMotorLeft;
    public double kP = 0.006;
    public double targetPos = 1600;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-14, 61, Math.toRadians(90)));
        hand = new EnableHand();
        out = new MiggyUnLimbetedOuttake();

        out.init(hardwareMap);
        hand.init(hardwareMap);

        Action highChamberTraj = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(0, 35, Math.toRadians(90)), Math.toRadians(270))
                .build();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 35, Math.toRadians(90)));


        Action obsZoneTraj = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-47, 59, Math.toRadians(270)), Math.toRadians(90))
                .build();
        drive = new MecanumDrive(hardwareMap, new Pose2d(-47, 59, Math.toRadians(270)));

        Action parkTraj = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-47, 59, Math.toRadians(90)), Math.toRadians(90))
                .build();




        while (!isStopRequested() && !opModeIsActive()) {
        }
        out.autonInit();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        highChamberTraj,
                        telemetryPacket -> {
                            telemetry.addLine("slides");
                            telemetry.update();
                            out.PIDLoop(640);
                            return false;
                        },
                        new SleepAction(0.7),

                        telemetryPacket -> {
                            telemetry.addLine("Arm");
                            telemetry.update();
                            out.Arm(0.6);
                            return false;
                        },
                        new SleepAction(0.3),

                        telemetryPacket -> {
                            telemetry.addLine("outtake flipper");
                            telemetry.update();
                            out.outtakeFlipper(0.0);
                            return false;
                        },
                        new SleepAction(0.4),

                        telemetryPacket -> {
                            telemetry.addLine("outtake claw");
                            telemetry.update();
                            out.transfer2();
                            return false;
                        },
                        new SleepAction(0.7),

                        telemetryPacket -> {
                            telemetry.addLine("set swing arm");
                            telemetry.update();
                            hand.setSwingArmAngleAuton(90);
                            return false;
                        },
                        new SleepAction(0.7),
                        telemetryPacket -> {
                            telemetry.addLine("drop slides");
                            telemetry.update();
                            out.PIDLoop(320);
                            return false;
                        },
                        new SleepAction(0.7),

                        telemetryPacket -> {
                            telemetry.addLine("score");
                            telemetry.update();
                            out.score();
                            return false;

                        },
                        new SleepAction(0.7),

                        telemetryPacket -> {
                            telemetry.addLine("bring down slides");
                            telemetry.update();
                            out.PIDLoop(0);
                            return false;
                        }
                        ));
    }
}
