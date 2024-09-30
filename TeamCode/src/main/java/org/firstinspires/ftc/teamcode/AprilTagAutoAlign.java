package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;

@Config
@Autonomous(name = "April Tag Auto Align")
public class AprilTagAutoAlign extends LinearOpMode {

    public static double turnGain = 0.03;
    public static double translateGain = 0.015;
    public static double strafeGain = 0.015;


    Drivetrain dt = new Drivetrain();



    @Override
    public void runOpMode() throws InterruptedException {
        dt.init(hardwareMap);

        dt.turnGain = turnGain;
        dt.translateGain = translateGain;
        dt.strafeGain = strafeGain;

        waitForStart();

        while (opModeIsActive()) {
            dt.alignAprilTag(12);
        }

    }
}
