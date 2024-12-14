package org.firstinspires.ftc.teamcode.RoadRunner.autos;

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

@Autonomous(name = "Specimen Auton Red")
public class REALSpecimenAutoRed extends LinearOpMode {
    Drivetrain drive = new Drivetrain();
    EnableHand hand = new EnableHand();
    MiggyUnLimbetedOuttake out = new MiggyUnLimbetedOuttake();

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive3 = new MecanumDrive(hardwareMap, new Pose2d(6.5, -60.75, Math.toRadians(270)));


        hand.init(hardwareMap);
        drive.init(hardwareMap);
        drive.initVisionPortal(hardwareMap);
        out.init(hardwareMap);
        out.transfer2();// CLOSES THE CLAW
        hand.setSwingArmAngleAuton(160);
        hand.open();


        Pose2d specimenScorePose = new Pose2d(9, -35, Math.toRadians(270));

        Pose2d sample1Pose = new Pose2d(42, -14, Math.toRadians(90));
        Pose2d sample1PushPose = new Pose2d(57, -60, Math.toRadians(90));

        Pose2d sample2Pose = new Pose2d(70, -14, Math.toRadians(90));
        Pose2d sample2PushPose = new Pose2d(65, -60, Math.toRadians(90));

        Pose2d sample3Pose = new Pose2d(72, -14, Math.toRadians(90));
        Pose2d sample3PushPose = new Pose2d(72, -60, Math.toRadians(90));

        Pose2d awaitingSpecimenPose = new Pose2d(47, -52, Math.toRadians(86));
        Pose2d parkPose = new Pose2d(47, -61, Math.toRadians(270));
        Pose2d specimenPickupPose= new Pose2d(47, -57, Math.toRadians(90));



        Action specimenScoreTraj1 = drive3.actionBuilder(drive3.pose).splineToLinearHeading(specimenScorePose, Math.toRadians(90)).build();
        drive3 = new MecanumDrive(hardwareMap, specimenScorePose);

        Action necessaryToNotHitSubmersible = drive3.actionBuilder(drive3.pose).lineToY(-39).build();
        drive3 = new MecanumDrive(hardwareMap, new Pose2d(9, -39, Math.toRadians(270)));


        Action sample1Traj = drive3.actionBuilder(drive3.pose).splineToLinearHeading(sample1Pose, Math.toRadians(90)).build();
        drive3 = new MecanumDrive(hardwareMap, sample1Pose);

        Action sample1PushTraj = drive3.actionBuilder(drive3.pose).splineToLinearHeading(sample1PushPose, Math.toRadians(90)).build();
        drive3 = new MecanumDrive(hardwareMap, sample1PushPose);


        Action sample2Traj = drive3.actionBuilder(drive3.pose).splineToLinearHeading(sample2Pose, Math.toRadians(10)).build();
        drive3 = new MecanumDrive(hardwareMap, sample2Pose);

        Action sample2PushTraj = drive3.actionBuilder(drive3.pose).splineToLinearHeading(sample2PushPose, Math.toRadians(90)).build();
        drive3 = new MecanumDrive(hardwareMap, sample2PushPose);


        Action sample3Traj = drive3.actionBuilder(drive3.pose).splineToLinearHeading(sample3Pose, Math.toRadians(10)).build();
        drive3 = new MecanumDrive(hardwareMap, sample3Pose);

        Action sample3PushTraj = drive3.actionBuilder(drive3.pose).splineToLinearHeading(sample3PushPose, Math.toRadians(90)).build();
        drive3 = new MecanumDrive(hardwareMap, sample3PushPose);



        Action awaitingSpecimenTraj = drive3.actionBuilder(drive3.pose).splineToLinearHeading(awaitingSpecimenPose, Math.toRadians(270)).build();
        drive3 = new MecanumDrive(hardwareMap, awaitingSpecimenPose);

        Action specimenPickUpTraj1 = drive3.actionBuilder(drive3.pose).splineToLinearHeading(specimenPickupPose, Math.toRadians(270)).build();
        drive3 = new MecanumDrive(hardwareMap, specimenPickupPose);

        Action specimenScoreTraj2 = drive3.actionBuilder(drive3.pose).splineToLinearHeading(specimenScorePose, Math.toRadians(90)).build();
        drive3 = new MecanumDrive(hardwareMap, specimenScorePose);



        Action awaitingSpecimenTraj2 = drive3.actionBuilder(drive3.pose).splineToLinearHeading(awaitingSpecimenPose, Math.toRadians(270)).build();
        drive3 = new MecanumDrive(hardwareMap, awaitingSpecimenPose);

        Action specimenPickUpTraj2 = drive3.actionBuilder(drive3.pose).splineToLinearHeading(specimenPickupPose, Math.toRadians(270)).build();
        drive3 = new MecanumDrive(hardwareMap, specimenPickupPose);

        Action specimenScoreTraj3 = drive3.actionBuilder(drive3.pose).splineToLinearHeading(specimenScorePose, Math.toRadians(90)).build();
        drive3 = new MecanumDrive(hardwareMap, specimenScorePose);


        Action awaitingSpecimenTraj3 = drive3.actionBuilder(drive3.pose).splineToLinearHeading(awaitingSpecimenPose, Math.toRadians(270)).build();
        drive3 = new MecanumDrive(hardwareMap, awaitingSpecimenPose);

        Action specimenPickUpTraj3 = drive3.actionBuilder(drive3.pose).splineToLinearHeading(specimenPickupPose, Math.toRadians(270)).build();
        drive3 = new MecanumDrive(hardwareMap, specimenPickupPose);

        Action specimenScoreTraj4 = drive3.actionBuilder(drive3.pose).splineToLinearHeading(specimenScorePose, Math.toRadians(90)).build();
        drive3 = new MecanumDrive(hardwareMap, specimenScorePose);


        Action preLoadPlace = new SequentialAction(
                new ParallelAction(
                        specimenScoreTraj1,
                        telemetryPacket -> {
                            telemetry.addData("Specimen slides pose", out.currentPos);
                            out.specimenSlideUp_noadjust( );
                            return false;
                        }
                ),
                new SleepAction(0.5),
                telemetryPacket -> {
                    telemetry.addData("Specimen slides pose", out.currentPos);
                    out.specimenSlideDown();
                    return false;
                },
                telemetryPacket -> {
                    telemetry.addData("Specimen slides pose", out.currentPos);
                    out.specimenRelease();
                    return false;
                }


        );


        Action push3Samples = new SequentialAction(
                sample1Traj,
                sample1PushTraj,
                sample2Traj,
                sample2PushTraj,
                sample3Traj,
                sample3PushTraj
        );

        while (!isStopRequested() && !opModeIsActive()) {}


        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                specimenScoreTraj1,
                new SleepAction(2),
                necessaryToNotHitSubmersible,
                sample1Traj,
                sample1PushTraj,
                sample2Traj,
                sample2PushTraj,
                sample3Traj,
                sample3PushTraj,
                awaitingSpecimenTraj,
                new SleepAction(1),
                specimenPickUpTraj1,
                new SleepAction(1),
                specimenScoreTraj2,
                new SleepAction(2),
                specimenPickUpTraj2,
                new SleepAction(1),
                specimenScoreTraj3,
                new SleepAction(2),
                specimenPickUpTraj3,
                new SleepAction(1),
                specimenScoreTraj4,
                new SleepAction(2)
        ));
    }
}
