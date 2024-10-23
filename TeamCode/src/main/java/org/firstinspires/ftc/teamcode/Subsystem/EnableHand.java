package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class EnableHand implements Subsystem {

    public Servo Claw, LServoSwingArm, RServoSwingArm, ArmTurr, ClawTurr;

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
        ArmTurr.setPosition(0.44);
    }






    public void scan1() {
        // Swing arm to 60 degrees
        setSwingArmAngle(60);
    }

    public void scan2() {
        // Intake arm turret: Intaking position
        ArmTurr.setPosition(0.44);
    }

    public void scan3() {
        // Claw in vectoring position
        open();
    }

    public void scan4() {
        // Hand turret angle to 90 degrees (TBD)
        ClawTurr.setPosition(setHandTurretDegrees(90)); // Placeholder for TBD
    }

    public void hover() {
        // Swing arm angle at 15 degrees
        setSwingArmAngle(15);
    }

    public void pickup1() {
        // Swing arm angle at -5 degrees
        setSwingArmAngle(-5);
    }

    public void pickup2() {
        // Claw in latching position
        close();
    }

    public void transfer1() {
        // Swing arm angle to 175 degrees
        setSwingArmAngle(100);
    }

    public void transfer1point5() {
        setSwingArmAngle(160);
    }


    public void transfer2() {
        // Hand turret to 0 degrees
        ClawTurr.setPosition(setHandTurretDegrees(0));
    }

    public void loiter() {
        // Claw in vectoring position
        open();
    }


}
