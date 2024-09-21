package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.Robot;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    // NEEDS TO BE TESTED

    Drivetrain drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain();
        drive.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            drive.TeleopControl(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
        }

    }
}