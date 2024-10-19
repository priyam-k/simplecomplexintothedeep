package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "ServoAndPortTest")
public class ServoAndPortTester extends LinearOpMode {
    public static double CtrlServo0Pos = 0.44;
    public static double CtrlServo1Pos = 0;
    public static double CtrlServo2Pos = 0;
    public static double CtrlServo3Pos = 0.97;
    public static double CtrlServo4Pos = 0;
    public static double Servo5Pos = 0;
    public static double ExpServo0Pos = 0;
    public static double ExpServo1Pos = 0;
    public static double ExpServo2Pos = 0;
    public static double ExpServo3Pos = 0.43;
    public static double ExpServo4Pos = 0.43;
    public static double ExpServo5Pos = 0;
    public static double SwingArmAngleDegrees = 0;
    public static double HandTurretAngleDegrees = 0;

    public static double OuttakeArmTicks = 0.0;
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
        ExpansionHub0 = hardwareMap.get(Servo.class, "Servo6");
        ExpansionHub1 = hardwareMap.get(Servo.class, "Servo7");
        ExpansionHub2 = hardwareMap.get(Servo.class, "Servo8");
        ExpansionHub3 = hardwareMap.get(Servo.class, "Servo9");
        ExpansionHub4 = hardwareMap.get(Servo.class, "Servo10");
        ExpansionHub5 = hardwareMap.get(Servo.class, "Servo11");

        waitForStart();

        while (opModeIsActive()) {
            // Set and display positions for Control Hub servos
            ControlHub0.setPosition(CtrlServo0Pos);
            telemetry.addData("ControlHub0 Position", CtrlServo0Pos);

            ControlHub1.setPosition(CtrlServo1Pos);
            telemetry.addData("ControlHub1 Position", CtrlServo1Pos);


            telemetry.addData("Outtake arm(2 and3) ControlHub", OuttakeArmTicks);
            ControlHub2.setPosition(OuttakeArmTicks);
            ControlHub3.setPosition(OuttakeArmTicks);

            ControlHub4.setPosition(CtrlServo4Pos);
            telemetry.addData("ControlHub4 Position", CtrlServo4Pos);

            ControlHub5.setPosition(Servo5Pos);
            telemetry.addData("ControlHub5 Position", Servo5Pos);

            // Set and display positions for Expansion Hub servos


            // Hand Turret Position
            ExpServo0Pos = degreesToTicks(HandTurretAngleDegrees);
            ExpansionHub0.setPosition(ExpServo0Pos);
            telemetry.addData("Hand Turret (Exp0) Position", ExpServo0Pos);

            // Swing Arm Position
            ExpServo1Pos = degreesToTicksSwingArm(SwingArmAngleDegrees);
            ExpansionHub1.setPosition(ExpServo1Pos);
            ExpansionHub2.setPosition(ExpServo1Pos);
            telemetry.addData("Swing Arm (Exp1) Position", ExpServo1Pos);

            ExpansionHub3.setPosition(ExpServo3Pos);
            telemetry.addData("ExpansionHub3 Position", ExpServo3Pos);

            ExpansionHub4.setPosition(ExpServo4Pos);
            telemetry.addData("ExpansionHub4 Position", ExpServo4Pos);

            ExpansionHub5.setPosition(ExpServo5Pos);
            telemetry.addData("ExpansionHub5 Position", ExpServo5Pos);


            telemetry.update();
        }
    }

    public double degreesToTicksSwingArm(double d) {
        d = -d;
        return d / 355 + 0.963;
    }

    public double degreesToTicksHandTurret(double d) {
        d = -d;
        return d / 327;
    }


    public double degreesToTicks(double d){
        return d/270;
    }
}
