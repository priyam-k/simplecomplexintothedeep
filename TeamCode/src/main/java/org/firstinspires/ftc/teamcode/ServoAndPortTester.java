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
@TeleOp(name = "ServoAndPortTest")
public class ServoAndPortTester extends LinearOpMode {


    public static double OuttakeClaw = 0.44;
    public static double OuttakeFlipper = 0.4;
    //public static double CtrlServo2Pos = 0;
    //public static double CtrlServo3Pos = 0.97;
    public static double CtrlServo4Pos = 0;
    public static double Servo5Pos = 0;
    public static double ExpServo0Pos = 0;
    public static double ExpServo1Pos = 0;
   // public static double ExpServo2Pos = 0;
    public static double IntakeVectoringHand = 0.43;
    public static double IntakeArmTurret = 0.43;
    public static double ExpServo5Pos = 0;
        public static double SwingArmAngleDegrees = 90;
    public static double HandTurretAngleDegrees = 174;
    public static double offset = 186;

    public static double OuttakeArmTicks = 0.7;
    private DcMotorEx slideMotorRight;
    private DcMotorEx slideMotorLeft;

    public static double Kp =0.03, targetPos =0.0;
    private MultipleTelemetry tele;
    private Servo ControlHub0;
    private Servo ControlHub1;
    private Servo ControlHub2;
    private Servo ControlHub3;
    private Servo ControlHub4;
    private Servo ControlHub5; // TODO make angle to tick thing, and have it set controlhub 3 and 5 to that angle
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

        slideMotorRight = hardwareMap.get(DcMotorEx.class, "rightLift");
        slideMotorLeft = hardwareMap.get(DcMotorEx.class, "leftLift");

        slideMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        slideMotorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            ControlHub0.setPosition(OuttakeClaw);
            telemetry.addData("Outtake claw", OuttakeClaw);
                                                           
            ControlHub1.setPosition(OuttakeFlipper);
            telemetry.addData("Flipper", OuttakeFlipper);


            telemetry.addData("Outtake arm(2 and3) ControlHub", OuttakeArmTicks);
            ControlHub2.setPosition(OuttakeArmTicks);
            ControlHub3.setPosition(OuttakeArmTicks);

            ControlHub4.setPosition(CtrlServo4Pos);
            telemetry.addData("ControlHub4 Position", CtrlServo4Pos);

            ControlHub5.setPosition(Servo5Pos);
            telemetry.addData("ControlHub5 Position", Servo5Pos);

            // Set and display positions for Expansion Hub servos


            // Hand Turret Position
            ExpServo0Pos = setHandTurretDegrees(HandTurretAngleDegrees); //degreesToTicks(HandTurretAngleDegrees);
            ExpansionHub0.setPosition(ExpServo0Pos);
            telemetry.addData("Hand Turret (Exp0) Position", ExpServo0Pos);

            // Swing Arm Position
            ExpServo1Pos = degreesToTicksSwingArm(SwingArmAngleDegrees);
            ExpansionHub1.setPosition(ExpServo1Pos);
            ExpansionHub2.setPosition(ExpServo1Pos);
            telemetry.addData("Swing Arm (Exp1and2) Position", ExpServo1Pos);

            ExpansionHub3.setPosition(IntakeVectoringHand);
            telemetry.addData("Vectoring hand", IntakeVectoringHand);
            
            ExpansionHub4.setPosition(IntakeArmTurret);
            telemetry.addData("Arm Turret", IntakeArmTurret);

            ExpansionHub5.setPosition(ExpServo5Pos);
            telemetry.addData("ExpansionHub5 Position", ExpServo5Pos);




            // obtain the encoder position
            double encoderPositionRight = slideMotorRight.getCurrentPosition();
            // calculate the error
            double error = targetPos - encoderPositionRight;
            //
            double out = -(Kp * error) ;
            //1428 max

            slideMotorRight.setPower(out);
            slideMotorLeft.setPower(out);
            tele.addData("The motor power is: ", out);
            tele.addData("Current position Slides: ", encoderPositionRight);
            tele.addData("Target position Slides: ", targetPos);
            tele.update();
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

    // only do degree values between -90 and 180 otherwise it will explode
    public double setHandTurretDegrees(double d){
        double t = (((d-offset)/270.0)+3)%1;
        telemetry.addData("ticks", t);
        telemetry.addData("degrees", d-180);
        return t;

    }


    public double degreesToTicks(double d){
        return d/270;
    }
}
