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
    private Servo ControlHub0;
    private Servo ControlHub1;
    private Servo ControlHub2;
    private Servo ControlHub3;
    private Servo ControlHub4;
    private Servo ControlHub5;
    private Servo ExpansionHub0;
    private Servo ExpansionHub1;
    private Servo ExpansionHub2;
    private Servo ExpansionHub3;
    private Servo ExpansionHub4;
    private Servo ExpansionHub5;

    public static double Servo0Pos = 0;
    public static double Servo1Pos = 0;
    public static double claw = 0;
    public static double swingArm = 0;
    public static double clawTurret = 0;

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

        // Initialize the servos for the Control Hub
        ControlHub0 = hardwareMap.get(Servo.class, "Servo0");
        ControlHub1 = hardwareMap.get(Servo.class, "Servo1");
        ControlHub2 = hardwareMap.get(Servo.class, "Servo2");
        ControlHub3 = hardwareMap.get(Servo.class, "Servo3");
        ControlHub4 = hardwareMap.get(Servo.class, "Servo4");
        ControlHub5 = hardwareMap.get(Servo.class, "Servo5");

        // Initialize the servos for the Expansion Hub
        ExpansionHub0  = hardwareMap.get(Servo.class, "Servo6");
        ExpansionHub1  = hardwareMap.get(Servo.class, "Servo7");
        ExpansionHub2  = hardwareMap.get(Servo.class, "Servo8");
        ExpansionHub3  = hardwareMap.get(Servo.class, "Servo9");
        ExpansionHub4 = hardwareMap.get(Servo.class, "Servo10");
        ExpansionHub5 = hardwareMap.get(Servo.class, "Servo11");

        waitForStart();

        while (opModeIsActive()) {
            // Set and display positions for Control Hub servos
            ControlHub0.setPosition(Servo0Pos);
            telemetry.addData("ControlHub0 Position", Servo0Pos);

            ControlHub1.setPosition(Servo1Pos);
            telemetry.addData("ControlHub1 Position", Servo1Pos);

            ControlHub2.setPosition(claw);
            telemetry.addData("ControlHub2 Position", claw);

            ControlHub3.setPosition(swingArm);
            telemetry.addData("ControlHub3 Position", swingArm);
            //right swing arm

            ControlHub4.setPosition(clawTurret);
            telemetry.addData("ControlHub4 Position", clawTurret);

            ControlHub5.setPosition(swingArm);
            telemetry.addData("ControlHub5 Position", swingArm);
            //left swing arm

            // Set and display positions for Expansion Hub servos
            ExpansionHub0.setPosition(Servo6Pos);
            telemetry.addData("ExpansionHub0 Position", Servo6Pos);

            ExpansionHub1.setPosition(turretPos);
            telemetry.addData("ExpansionHub1 Position", turretPos);

            ExpansionHub2.setPosition(Servo8Pos);
            telemetry.addData("ExpansionHub2 Position", Servo8Pos);

            ExpansionHub3.setPosition(Servo9Pos);
            telemetry.addData("ExpansionHub3 Position", Servo9Pos);

            ExpansionHub4.setPosition(Servo10Pos);
            telemetry.addData("ExpansionHub4 Position", Servo10Pos);

            ExpansionHub5.setPosition(Servo11Pos);
            telemetry.addData("ExpansionHub5 Position", Servo11Pos);

            telemetry.update();
        }
    }
}
