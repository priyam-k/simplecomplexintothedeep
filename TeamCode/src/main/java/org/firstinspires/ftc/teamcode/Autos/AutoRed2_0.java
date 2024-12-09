package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
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


@Config
@Autonomous(name = "A: Auto Red 2+0")
public class AutoRed2_0 extends LinearOpMode {
    Drivetrain drive = new Drivetrain();
    EnableHand hand = new EnableHand();
    MiggyUnLimbetedOuttake out = new MiggyUnLimbetedOuttake();


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(16.75, -60.75, Math.toRadians(270)));

        hand.init(hardwareMap);
        drive.init(hardwareMap);
        drive.initVisionPortal(hardwareMap);
        out.init(hardwareMap);


        out.transfer2();
        hand.setSwingArmAngleAuton(130);
        hand.open();

        Pose2d highChamberPose = new Pose2d(0, -35, Math.toRadians(270));
        Pose2d obsZonePose = new Pose2d(47, -52, Math.toRadians(86));
        Pose2d parkPose = new Pose2d(47, -61, Math.toRadians(270));
        Pose2d obsZonePickupPose = new Pose2d(47, -59, Math.toRadians(90));

        Action highChamberTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(highChamberPose, Math.toRadians(90)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, highChamberPose);

        Action obsZoneTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(obsZonePose, Math.toRadians(270)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, obsZonePickupPose);

        Action highChamberTraj2 = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(highChamberPose, Math.toRadians(90)).build();
        mecanumDrive = new MecanumDrive(hardwareMap, obsZonePose);

        Action obsZonePickupTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).lineToY(-59).build();
        mecanumDrive = new MecanumDrive(hardwareMap, highChamberPose);
        Action parkTraj = mecanumDrive.actionBuilder(mecanumDrive.pose).splineToLinearHeading(parkPose, Math.toRadians(270)).build();

        while (!isStopRequested() && !opModeIsActive()) {
        }

        waitForStart();
        out.autonInit();


        if (isStopRequested()) return;


        Action specimen1action = new SequentialAction(telemetryPacket -> {
            out.PIDLoop(800);

            telemetry.addLine("Slides 800");
            telemetry.addData("real pos", out.Rlift.getCurrentPosition());

            telemetry.update();
            return false;
        }, new SleepAction(5), telemetryPacket -> {
            out.PIDLoop(600);

            telemetry.addLine("Slides 600");
            telemetry.addData("real pos", out.Rlift.getCurrentPosition());

            telemetry.update();

            return false;
        }, new SleepAction(5));


        Actions.runBlocking(new SequentialAction(highChamberTraj, specimen1action, telemetryPacket -> {
            out.PIDLoop(-100);

            telemetry.addLine("Slides 0");
            telemetry.addData("real pos", out.Rlift.getCurrentPosition());
            telemetry.update();

            return false;
        }, new SleepAction(2), obsZoneTraj, new SleepAction(3), obsZonePickupTraj, highChamberTraj2, new SleepAction(3), parkTraj));

        telemetry.update();


    }
}
