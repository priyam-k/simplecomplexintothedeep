package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;

@Config
@Autonomous(name = "April Tag Auto Align")
public class AprilTagAutoAlign extends LinearOpMode {

    public static double turnGain = 0.04;
    public static double translateGain = 0.2;
    public static double strafeGain = 0.0001;

    public static double RandomdistanceUnits = 12.0;

    Drivetrain drive = new Drivetrain();
    EnableHand intake = new EnableHand();

    @Override
    public void runOpMode() throws InterruptedException {
        intake.init(hardwareMap);
        drive.init(hardwareMap);
        intake.setSwingArmAngle(90);
        while (opModeInInit()) {
            drive.turnGain = turnGain;
            drive.translateGain = translateGain;
            drive.strafeGain = strafeGain;

            drive.alignAprilTag(distance);
        }

        waitForStart();


    }
}
