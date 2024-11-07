package org.firstinspires.ftc.teamcode.SingleUse;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
//@TeleOp(name="EncoderTesterWithPIDFTuning")
public class EncoderTester extends LinearOpMode {

    // Declare motors
    private DcMotorEx rightFrontEnc = null;
    private DcMotorEx rightRearEnc = null;
    private DcMotorEx leftFrontEnc = null;
    private DcMotorEx leftRearEnc = null;

    // Proportional variables for each motor, accessible via FTC Dashboard
    public static double P_rightFront = 10.0;
    public static double P_rightRear = 10.0;
    public static double P_leftFront = 10.0;
    public static double P_leftRear = 10.0;

    @Override
    public void runOpMode() {

        // Initialize the motors as DcMotorEx
        rightFrontEnc = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRearEnc = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFrontEnc = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRearEnc = hardwareMap.get(DcMotorEx.class, "leftRear");



        // Set zero power behavior to BRAKE for all motors
        rightFrontEnc.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightRearEnc.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        leftFrontEnc.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        leftRearEnc.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // Reset the encoders before starting
        rightFrontEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run using encoders
        rightFrontEnc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearEnc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontEnc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearEnc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply the PIDF coefficients (only once) using the P values from the dashboard
        updatePIDFCoefficients();

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
            telemetry.update();
        }
    }

    // Method to update the PIDF coefficients for each motor using the individual P values
    private void updatePIDFCoefficients() {
        rightFrontEnc.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P_rightFront, 0, 0, 0));
        rightRearEnc.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P_rightRear, 0, 0, 0));
        leftFrontEnc.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P_leftFront, 0, 0, 0));
        leftRearEnc.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P_leftRear, 0, 0, 0));
    }
}
