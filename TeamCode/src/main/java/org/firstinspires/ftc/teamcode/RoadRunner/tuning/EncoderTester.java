package org.firstinspires.ftc.teamcode.RoadRunner.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name="EncoderTester")
public class EncoderTester extends LinearOpMode {

    // Declare motors
    private DcMotor rightFrontEnc = null;
    private DcMotor rightRearEnc = null;
    private DcMotor leftFrontEnc = null;
    private DcMotor leftRearEnc = null;

    @Override
    public void runOpMode() {

        // Initialize the motors
        rightFrontEnc = hardwareMap.get(DcMotor.class, "rightFront");
        rightRearEnc = hardwareMap.get(DcMotor.class, "rightRear");
        leftFrontEnc = hardwareMap.get(DcMotor.class, "leftFront");
        leftRearEnc = hardwareMap.get(DcMotor.class, "leftRear");


        // Reset the encoders before starting
        rightFrontEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run using encoders
        rightFrontEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the start button to be pressed
        waitForStart();

        // Run the motors for 5 seconds
        rightFrontEnc.setPower(0.5); // Set motor power to 50%
        rightRearEnc.setPower(0.5);
        leftFrontEnc.setPower(0.5);
        leftRearEnc.setPower(0.5);

        sleep(5000); // Run for 5 seconds (5000 milliseconds)

        // Stop the motors
        rightFrontEnc.setPower(0);
        rightRearEnc.setPower(0);
        leftFrontEnc.setPower(0);
        leftRearEnc.setPower(0);

        // Retrieve the encoder values
        int rightFrontTicks = rightFrontEnc.getCurrentPosition();
        int rightRearTicks = rightRearEnc.getCurrentPosition();
        int leftFrontTicks = leftFrontEnc.getCurrentPosition();
        int leftRearTicks = leftRearEnc.getCurrentPosition();

        // Display the tick values on the telemetry
        telemetry.addData("Right Front Ticks: ", rightFrontTicks);
        telemetry.addData("Right Rear Ticks: ", rightRearTicks);
        telemetry.addData("Left Front Ticks: ", leftFrontTicks);
        telemetry.addData("Left Rear Ticks: ", leftRearTicks);
        telemetry.update();

        // Keep displaying values until the OpMode ends
        while (opModeIsActive()) {
            // Just keep updating telemetry
            telemetry.update();
        }
    }
}
