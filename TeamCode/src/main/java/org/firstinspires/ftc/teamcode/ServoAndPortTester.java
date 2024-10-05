package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="ServoAndPortTest")
public class ServoAndPortTester extends LinearOpMode {
    private MultipleTelemetry tele;
    private Servo testServo0;
    private Servo testServo1;
    private Servo testServo2;
    private Servo testServo3;
    private Servo testServo4;
    private Servo testServo5;
    private Servo testServo6;
    private Servo testServo7;
    private Servo testServo8;
    private Servo testServo9;
    private Servo testServo10;
    private Servo testServo11;

    public static double Servo0Pos = 0;
    public static double Servo1Pos = 0;
    public static double claw = 0;
    public static double swingArm = 0;
    public static double clawTurret = 0;
    public static double rightArm = 0;

    public static double Servo6Pos = 0;
    public static double turretPos = 0;
    public static double Servo8Pos = 0;
    public static double Servo9Pos = 0;
    public static double Servo10Pos = 0;
    public static double Servo11Pos = 0;


    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize the servo

        // Control Hub
        testServo0 = hardwareMap.get(Servo.class, "Servo0");
        testServo1 = hardwareMap.get(Servo.class, "Servo1");
        testServo2 = hardwareMap.get(Servo.class, "Servo2");
        testServo3 = hardwareMap.get(Servo.class, "Servo3");
        testServo4 = hardwareMap.get(Servo.class, "Servo4");
        testServo5 = hardwareMap.get(Servo.class, "Servo5");

        // Expansion Hub
        testServo6  = hardwareMap.get(Servo.class, "Servo6");
        testServo7  = hardwareMap.get(Servo.class, "Servo7");
        testServo8  = hardwareMap.get(Servo.class, "Servo8");
        testServo9  = hardwareMap.get(Servo.class, "Servo1");
        testServo10 = hardwareMap.get(Servo.class, "Servo10");
        testServo11 = hardwareMap.get(Servo.class, "Servo11");


        waitForStart();

        while (opModeIsActive()) {
            // Servo 0 Pos vals
            testServo0.setPosition(Servo0Pos);
            telemetry.addData("Servo 0 Position", Servo0Pos);
            // Servo 1 Pos vals
            testServo1.setPosition(Servo1Pos);
            telemetry.addData("Servo 1 Position", Servo1Pos);
            // Servo 2 Pos vals
            testServo2.setPosition(claw);
            telemetry.addData("Servo 2 Position", claw);
            // Servo 3 Pos vals
            testServo3.setPosition(swingArm);
            testServo5.setPosition(swingArm);
            telemetry.addData("Servo 3 Position", swingArm);
            telemetry.addData("Servo 5 position", rightArm);
            // Servo 4 Pos vals
            testServo4.setPosition(clawTurret);
            telemetry.addData("Servo 4 Position", clawTurret);
            // Servo 6 Pos vals
            testServo6.setPosition(Servo6Pos);
            telemetry.addData("Servo 6 Position", Servo6Pos);
            // Servo 7 Pos vals
            testServo7.setPosition(turretPos);
            telemetry.addData("Servo 7 Position", turretPos);
            // Servo 8 Pos vals
            testServo8.setPosition(Servo8Pos);
            telemetry.addData("Servo 8 Position", Servo8Pos);
            // Servo 9 Pos vals
            testServo9.setPosition(Servo9Pos);
            telemetry.addData("Servo 9 Position", Servo9Pos);
            // Servo 10 Pos vals
            testServo10.setPosition(Servo10Pos);
            telemetry.addData("Servo 10 Position", Servo10Pos);
            // Servo 11 Pos vals
            testServo11.setPosition(Servo11Pos);
            telemetry.addData("Servo 11 Position", Servo11Pos);

            telemetry.update();
        }
    }
}