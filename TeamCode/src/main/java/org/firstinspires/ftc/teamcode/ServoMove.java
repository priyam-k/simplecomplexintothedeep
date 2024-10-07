package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * Illustration to show how to use a "minimal OpMode" class to call generic methods in other
 * classes.  The intent is to develop the generic methods to work for either Red or Blue Alliance
 * sides.  Which one actually runs is determined by this calling OpMode.
 */
@TeleOp(name = "Run Servo")
//@Disabled

public class ServoMove extends LinearOpMode {

    Servo servo;
    static final int ITERATIONS = 100;
    static final long DELAY = 5000;
    int i;  // looping index


    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.servo.get("SV");

        waitForStart();

        i = 0;
        servo.setPosition(0.55);
        sleep(2000);
        while(i++<ITERATIONS && opModeIsActive()){
            telemetry.addData("Cycle: ", i);
            telemetry.update();
            servo.setPosition(0.0);
            sleep(DELAY);
            servo.setPosition(1.0);
            sleep(DELAY);
        }

    }

}
