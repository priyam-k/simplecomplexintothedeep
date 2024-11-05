package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;

public class AdiRunnerTuner extends LinearOpMode {
    public static double KpVert, KpStraffe, KpRoatation;
    private IMU imu;
    private YawPitchRollAngles angles;
    private Drivetrain drive;

    @Override
    public void runOpMode() throws InterruptedException {
        Init(); //fix the imu

        waitForStart();

        while (opModeIsActive()){

        }



    }

    public void Init(){
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        //TODO FIX THIS PART
        imu.resetYaw();
        angles = imu.getRobotYawPitchRollAngles();

        drive = new Drivetrain();
        drive.init(hardwareMap);
    }
}
