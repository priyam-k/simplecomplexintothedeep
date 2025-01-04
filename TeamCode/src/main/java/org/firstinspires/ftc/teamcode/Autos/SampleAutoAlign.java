package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.teamcode.Subsystem.MiggyUnLimbetedOuttake;
import org.firstinspires.ftc.teamcode.Subsystem.Robot;

@Autonomous(name = "Auto Align Sample Auto")
public class SampleAutoAlign extends LinearOpMode {
    Drivetrain drive = new Drivetrain();
    EnableHand hand = new EnableHand();
    MiggyUnLimbetedOuttake out = new MiggyUnLimbetedOuttake();
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive3 = new MecanumDrive(hardwareMap, new Pose2d(-32.3, -64, Math.toRadians(90)));


        hand.init(hardwareMap);
        drive.init(hardwareMap);
        drive.initVisionPortal(hardwareMap);
        robot.init(hardwareMap);
        out.init(hardwareMap);

        out.transfer2(); // CLOSES THE CLAW
        hand.setSwingArmAngleAuton(130);
        hand.open();


        Pose2d basketPose = new Pose2d(-56, -53, Math.toRadians(45));
        Pose2d basketPose2 = new Pose2d(-57, -53, Math.toRadians(45));
        Pose2d basketPose3 = new Pose2d(-58, -53, Math.toRadians(45));
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


        // auto goes here
    }
}
