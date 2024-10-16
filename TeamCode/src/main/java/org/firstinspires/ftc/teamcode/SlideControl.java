package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="SlideControl", group="Linear Opmode")
public class SlideControl extends LinearOpMode {


    private DcMotor slideMotorLeft;
    private DcMotor slideMotorRight;



    @Override
    public void runOpMode() {
        // Initialize the hardware
        //TODO: Change theese configs
        slideMotorRight = hardwareMap.get(DcMotor.class, "rightLift");
        slideMotorLeft = hardwareMap.get(DcMotor.class, "leftLift");

        slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set motor direction if needed
        //TODO: reverse on of theese
        slideMotorRight.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

                slideMotorRight.setPower(gamepad1.left_stick_y);
                slideMotorLeft.setPower(gamepad1.left_stick_y);


            // Show the motor power in telemetry
            telemetry.addData("Slide Motor Power", slideMotorRight.getPower());
            telemetry.update();
        }
    }
}
