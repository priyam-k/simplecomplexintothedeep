package org.firstinspires.ftc.teamcode.SingleUse;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name = "Encoder Debugger")
public class EncoderDebugger extends LinearOpMode {
    // Define motors (or encoders) here
    private DcMotor lodo;
    private DcMotor rodo;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        rodo = hardwareMap.get(DcMotor.class, "Rodo");

        lodo = hardwareMap.get(DcMotor.class, "Lodo");

        // Reset encoders to zero if needed
        lodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run using encoders
        lodo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rodo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the start command from the driver station
        waitForStart();

        while (opModeIsActive()) {
            // Fetch and display encoder positions
            int leftPosition = lodo.getCurrentPosition();
            int rightPosition = rodo.getCurrentPosition();

            telemetry.addData("Left Encoder Ticks", leftPosition);
            telemetry.addData("Right Encoder Ticks", rightPosition);
            telemetry.update();

        }
    }
}
