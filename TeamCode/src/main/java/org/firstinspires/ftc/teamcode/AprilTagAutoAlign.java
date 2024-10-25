package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.teamcode.Subsystem.MiggyUnLimbetedOuttake;

@Config
@Autonomous(name = "April Tag Auto Align")
public class AprilTagAutoAlign extends LinearOpMode {

    public static double turnGain = 0.04;
    public static double translateGain = 0.2;
    public static double strafeGain = 0.01;

    public static double RandomdistanceUnits = 24.0;

    Drivetrain drive = new Drivetrain();
    EnableHand intake = new EnableHand();
    MiggyUnLimbetedOuttake outake = new MiggyUnLimbetedOuttake();

    @Override
    public void runOpMode() throws InterruptedException {
        intake.init(hardwareMap);
        drive.init(hardwareMap);
        outake.init(hardwareMap);

        intake.setSwingArmAngle(90);
        outake.autonInit();


        waitForStart();
        drive.drive(1000,0.5);
        while(opModeIsActive()) {

            drive.turnGain = turnGain;
            drive.translateGain = translateGain;
            drive.strafeGain = strafeGain;

            drive.alignAprilTag(RandomdistanceUnits);
        }
    }
}
