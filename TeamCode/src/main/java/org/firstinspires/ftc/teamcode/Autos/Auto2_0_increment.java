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


@Config
@Autonomous(name = "A: Auto 2+0: Adaptable")
public class Auto2_0_increment extends LinearOpMode {
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


        out.transfer2();
        out.specimenAutonInit();
        hand.setSwingArmAngleAuton(130);
        hand.open();

        double offset_chamber = 0;
        double offset2_pickup = 0;
        boolean[] prev_dpad = {false, false, false, false}; // previous dpad values for rising edge detection

        while (!isStopRequested() && !opModeIsActive()) {
            // offset control before start, up/down is chamber, left/right is pickup
            if (gamepad1.dpad_up && !prev_dpad[0]){
                prev_dpad[0] = true;
                offset_chamber += 0.5;
            } else if (gamepad1.dpad_down && !prev_dpad[2]){
                prev_dpad[2] = true;
                offset_chamber -= 0.5;
            }

            if (gamepad1.dpad_right && !prev_dpad[1]){
                prev_dpad[1] = true;
                offset2_pickup += 0.5;
            } else if (gamepad1.dpad_left && !prev_dpad[1]){
                prev_dpad[3] = true;
                offset2_pickup -= 0.5;
            }

            if (gamepad1.a){
                break;
            }
        }


        Pose2d highChamberPose = new Pose2d(0, -33.5, Math.toRadians(270));
        Pose2d highChamberPose2 = new Pose2d(5, -57.5 , Math.toRadians(270));
        Pose2d sample1Pose = new Pose2d(44.5, 0, Math.toRadians(45));
        Pose2d sample1PushPose = new Pose2d(44.5, -60, Math.toRadians(90));
        Pose2d sample2Pose = new Pose2d(60, -14, Math.toRadians(90));
        Pose2d sample2PushPose = new Pose2d(60, -60, Math.toRadians(90));
        Pose2d obsZonePose = new Pose2d(47, -52, Math.toRadians(90));
        Pose2d parkPose = new Pose2d(47, -61, Math.toRadians(270));
        Pose2d obsZonePickupPose = new Pose2d(50, -85, Math.toRadians(90));

        Action highChamberTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(highChamberPose, Math.toRadians(90)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, highChamberPose);

        Action sample1Traj = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(sample1Pose, Math.toRadians(75)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, sample1Pose);

        Action sample1PushTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(sample1PushPose, Math.toRadians(90)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, sample1PushPose);

        Action sample2Traj = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(sample2Pose, Math.toRadians(0)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, sample2Pose);

        Action sample2PushTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(sample2PushPose, Math.toRadians(90)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, sample2PushPose);

        Action obsZoneTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(obsZonePose, Math.toRadians(270)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, obsZonePose);

        Action obsZonePickupTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(obsZonePickupPose, Math.toRadians(90)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, obsZonePickupPose);


        Action highChamberTraj2 = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(highChamberPose2, Math.toRadians(90)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, highChamberPose);




        Action parkTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(parkPose, Math.toRadians(270)).build();


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
                                }),
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
                        sample1Traj,
                        telemetryPacket -> {
                            out.PIDLoop(0);
                            return conditionalEnd(0);
                        },
                        telemetryPacket -> {
                            out.specimenPickupStart();
                            return false;
                        },
                        sample1PushTraj,
                        sample2Traj,
                        sample2PushTraj,
                        obsZoneTraj,
                        obsZonePickupTraj,
                        new SleepAction(1),
                        telemetryPacket -> {
                            out.specimenPickupGrab();
                            return false;
                        },
                        new SleepAction(0.5),
                        telemetryPacket -> {
                            out.specimenPickupUp();
                            out.PIDLoop(100);
                            return conditionalEnd(100);
                        },
                        new SleepAction(0.5),

                        new ParallelAction(
                                highChamberTraj2, telemetryPacket ->{
                            out.PIDLoop(900);
                            out.outtakeFlipper.setPosition(0.4);

                            return conditionalEnd(900);
                        }),
                        new SleepAction(1),
                        telemetryPacket -> {
                            out.PIDLoop(500);
                            return conditionalEnd(500);
                        },
                        new SleepAction(0.5),
                        telemetryPacket -> {
                            out.specimenRelease();
                            return false;
                        },
                        new SleepAction(0.5),
                        telemetryPacket -> {
                            out.PIDLoop(0);
                            return conditionalEnd(0);
                        },
                        new SleepAction(0.5),

                        parkTraj
                ));

        telemetry.update();


    }
}
