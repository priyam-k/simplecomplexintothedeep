package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Servo Test")
public class ServoAndPortTester extends LinearOpMode {
    private MultipleTelemetry tele;
    private Servo testServo0;
    private Servo testServo1;
    private Servo testServo2;
    private Servo testServo3;
    private Servo testServo4;
    private Servo testServo5;
    public static double Servo0Pos = 0;
    public static double Servo1Pos = 0;
    public static double Servo2Pos = 0;
    public static double Servo3Pos = 0;
    public static double Servo4Pos = 0;
    public static double Servo5Pos = 0;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize the servo
        testServo0 = hardwareMap.get(Servo.class, "servo0");
        testServo1 = hardwareMap.get(Servo.class, "servo1");
        testServo2 = hardwareMap.get(Servo.class, "servo2");
        testServo3 = hardwareMap.get(Servo.class, "servo3");
        testServo4 = hardwareMap.get(Servo.class, "servo4");
        testServo5 = hardwareMap.get(Servo.class, "servo5");
        waitForStart();

        while (opModeIsActive()) {
            // Servo 0 Pos vals
            testServo0.setPosition(Servo0Pos);
            telemetry.addData("Servo 0 Position", Servo0Pos);
            // Servo 1 Pos vals
            testServo1.setPosition(Servo1Pos);
            telemetry.addData("Servo 1 Position", Servo1Pos);
            // Servo 2 Pos vals
            testServo2.setPosition(Servo2Pos);
            telemetry.addData("Servo 2 Position", Servo2Pos);
            // Servo 3 Pos vals
            testServo3.setPosition(Servo3Pos);
            telemetry.addData("Servo 3 Position", Servo3Pos);
            // Servo 4 Pos vals
            testServo4.setPosition(Servo4Pos);
            telemetry.addData("Servo 4 Position", Servo4Pos);
            // Servo 5 Pos vals
            testServo5.setPosition(Servo5Pos);
            telemetry.addData("Servo 5 Position", Servo5Pos);
            telemetry.update();
        }
    }
}

