package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.teamcode.Subsystem.Robot;

@TeleOp
@Config
public class MecanumTeleOp extends LinearOpMode {
    // NEEDS TO BE TESTED

    Drivetrain drive;
    EnableHand Hand;

    public static double both = 0;

    private boolean waspressed = false;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain();
        Hand = new EnableHand();
        drive.init(hardwareMap);
        Hand.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            drive.TeleopControl(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
            Hand.BothArm(both);
            if (gamepad1.a){waspressed = true;}
            if (!gamepad1.a && waspressed){
                Hand.close();
                waspressed = false;}


        }//opmode end

    }
}