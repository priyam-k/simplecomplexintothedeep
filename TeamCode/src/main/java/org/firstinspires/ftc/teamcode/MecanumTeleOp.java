package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystem.Robot;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    // NEEDS TO BE TESTED

    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot();
        robot.init(hardwareMap,gamepad1,gamepad2);
        waitForStart();
        while (opModeIsActive()){
            robot.Drive();
        }

    }
}