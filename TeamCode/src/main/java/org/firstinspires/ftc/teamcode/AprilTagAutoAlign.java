package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;

@Config
@Autonomous(name = "April Tag Auto Align")
public class AprilTagAutoAlign extends LinearOpMode {

    public static double turnGain = 0.03;
    public static double translateGain = 0.05;
    public static double strafeGain = 0.03;

    Drivetrain drive = new Drivetrain();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);


        while (opModeInInit()) {

            drive.turnGain = turnGain;
            drive.translateGain = translateGain;
            drive.strafeGain = strafeGain;

            drive.alignAprilTag(24);

        }

        waitForStart();


    }
}
