package org.firstinspires.ftc.teamcode.SingleUse;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class REVLightsOpmode extends LinearOpMode {


    private DigitalChannel redLED;
    private DigitalChannel greenLED;
    private DigitalChannel yellowLED;


    @Override
    public void runOpMode() throws InterruptedException {

        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");
        yellowLED = hardwareMap.get(DigitalChannel.class, "yellow");

        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a)
                redLED.setState(true);
            redLED.setState(false);

            if (gamepad1.b)
                greenLED.setState(true);
            greenLED.setState(false);

            if (gamepad1.x)
                yellowLED.setState(true);
            yellowLED.setState(false);

        }



    }
}
