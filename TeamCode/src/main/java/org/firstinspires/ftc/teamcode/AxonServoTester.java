package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Axon Servo Position Tester")
public class AxonServoTester extends LinearOpMode{
    public static double servoPosition = 0.5;
    public static double servoDegreesInput = 0;
    private Servo servo;
    private MultipleTelemetry tele;
    public static double axonDegreeToTick = servoDegreesInput/servoPosition;

    @Override
    public void runOpMode(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        servo = hardwareMap.get(Servo.class, "Servo1");
        waitForStart();
        while(opModeIsActive()){
            servo.setPosition(servoPosition);
            axonDegreeToTick = servoDegreesInput/servoPosition;
            tele.addData("Servo Position", servo.getPosition());
            tele.addData("Servo Position (Degrees)", servoPosition * axonDegreeToTick);
            tele.addData("axon degree factor", axonDegreeToTick);
            tele.update();
        }
    }

    private double axonDegreesToTicks(double d) {
        return d / axonDegreeToTick;
    }
}
