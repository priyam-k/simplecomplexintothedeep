package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class EnableHand implements Subsystem {

    public Servo Claw, LServoSwingArm, RServoSwingArm, ArmTurr, ClawTurr;
    Gamepad gamepad;

    boolean wasPressed = false;
    boolean wasPressedR = false;
    boolean waspressedGP = false;

    private double ANGLE = 90;

    /*
    Latching on to the sample: 0.43
    Vectoring position: 0.6
     */
    @Override
    public void init(HardwareMap hardwareMap) {
        Claw = hardwareMap.get(Servo.class, "Servo9");
        LServoSwingArm = hardwareMap.get(Servo.class, "Servo7"); // left servo for intake swing arm
        RServoSwingArm = hardwareMap.get(Servo.class, "Servo8"); // right servo for intake swing arm
        ArmTurr = hardwareMap.get(Servo.class, "Servo10");
        ClawTurr = hardwareMap.get(Servo.class, "Servo6");
    }
    public void init(HardwareMap hardwareMap,Gamepad g2) {
        Claw = hardwareMap.get(Servo.class, "Servo9");
        LServoSwingArm = hardwareMap.get(Servo.class, "Servo7"); // left servo for intake swing arm
        RServoSwingArm = hardwareMap.get(Servo.class, "Servo8"); // right servo for intake swing arm
        ArmTurr = hardwareMap.get(Servo.class, "Servo10");
        ClawTurr = hardwareMap.get(Servo.class, "Servo6");
        gamepad = g2;
    }
    @Override
    public void update() {

    }

    @Override
    public void addTelemetry(Telemetry t) {

    }

    public static double offset = 186;

    // only do degree values between -90 and 180 otherwise it will explode
    public double setHandTurretDegrees(double d){
        return (((d-offset)/270.0)+3)%1;

    }

    private double degreesToTicks(double d) {
        return d / 270;
    }

    public void open() {
        Claw.setPosition(0.63);
    }

    public void close() {
        Claw.setPosition(0.44);
    }

    public double degreesToTicksSwingArm(double d) {
        d = -d;
        return d / 355 + 0.963;
    }




    public void setSwingArmAngle(double angle) {
        double swingArmAngle = degreesToTicksSwingArm(angle);
        LServoSwingArm.setPosition(swingArmAngle);
        RServoSwingArm.setPosition(swingArmAngle);
        ArmTurr.setPosition(0.43);
    }

    public void setSwingArmAngleAuton(double angle) {
        double swingArmAngle = degreesToTicksSwingArm(angle);
        LServoSwingArm.setPosition(swingArmAngle);
        RServoSwingArm.setPosition(swingArmAngle);
        ArmTurr.setPosition(0.45);
    }


    public void scan1() {
        setSwingArmAngle(60);  // Set swing arm to 60 degrees
        ArmTurr.setPosition(0.44); // Default intake arm turret position
        open(); // Ensure claw is open
        ClawTurr.setPosition(setHandTurretDegrees(90)); // Default claw turret angle
    }

    public void scan2() {
        setSwingArmAngle(60);  // Ensure swing arm is at 60 degrees
        ArmTurr.setPosition(0.44); // Intake arm turret in position for scanning
        open(); // Claw open
        ClawTurr.setPosition(setHandTurretDegrees(90)); // Default claw turret angle
    }

    public void scan3() {
        setSwingArmAngle(60);  // Ensure swing arm is at 60 degrees
        ArmTurr.setPosition(0.44); // Arm turret position remains the same
        open(); // Claw in vectoring (open) position
        ClawTurr.setPosition(setHandTurretDegrees(90)); // Claw turret angle at 90 degrees
    }

    public void scan4() {
        setSwingArmAngle(60);  // Swing arm angle remains at 60 degrees
        ArmTurr.setPosition(0.44); // Intake position
        open(); // Claw remains open
        ClawTurr.setPosition(setHandTurretDegrees(90)); // Hand turret angle to 90 degrees
    }

    public void hover1() {
        setSwingArmAngle(15); // Set swing arm angle to 15 degrees
        ArmTurr.setPosition(0.44); // Default arm turret position
        open(); // Ensure claw is open
    }

    public void hover2() {
        hover1(); // Ensure base hover position

        // Adjust claw position based on D-Pad input
        if (gamepad.dpad_left) {
            ClawTurr.setPosition(ClawTurr.getPosition() + 0.05);
        } else if (gamepad.dpad_right) {
            ClawTurr.setPosition(ClawTurr.getPosition() - 0.05);
        }
    }

    public void pickup1() {
        setSwingArmAngle(-5);  // Set swing arm to -5 degrees for pickup
        ArmTurr.setPosition(0.44); // Default intake turret position
    }

    public void pickup2() {
        pickup1(); // Ensure pickup1 position
        close(); // Claw closed for latching

    }

    public void transfer1() {
        setSwingArmAngle(100);  // Set swing arm to 100 degrees for transfer
        ArmTurr.setPosition(0.44); // Default intake position
        open(); // Claw open
    }

    public void transfer1point5() {
        setSwingArmAngle(160);  // Adjust swing arm to 160 degrees
        ArmTurr.setPosition(0.44); // Default intake position
        open(); // Claw open
    }

    public void transfer2() {
        setSwingArmAngle(160); // Ensure swing arm is at 160 degrees
        ClawTurr.setPosition(setHandTurretDegrees(0)); // Hand turret to 0 degrees
        open(); // Claw open
    }

    public void loiter() {
        setSwingArmAngle(60); // Default position
        ArmTurr.setPosition(0.44); // Intake position
        open(); // Claw open in vectoring position
        ClawTurr.setPosition(setHandTurretDegrees(90)); // Default claw turret angle
    }



}
