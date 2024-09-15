package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTesterOpmode extends LinearOpMode {

    private static final double SERVO_TARGET_POS = 0;
    FtcDashboard ftcDashboard;
    public static String name;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo= hardwareMap.servo.get(name);

        ftcDashboard= FtcDashboard.getInstance();

        MultipleTelemetry tele= new MultipleTelemetry(ftcDashboard.getTelemetry(), telemetry);

        while(opModeIsActive()){
            servo.setPosition(SERVO_TARGET_POS);
            tele.addData("Target position: ", SERVO_TARGET_POS);
            tele.update();
        }
    }
}
