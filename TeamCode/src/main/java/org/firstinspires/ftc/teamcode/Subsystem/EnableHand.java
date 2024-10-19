package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class EnableHand implements Subsystem {

    private Servo Claw, LServoSwingArm, RServoSwingArm, ArmTurr, ClawTurr;

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
    public void addTelemtry(Telemetry t) {

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


    void setSwingArmAngle(double angle) {
        double swingArmAngle = degreesToTicks(angle);
        LServoSwingArm.setPosition(swingArmAngle);
        RServoSwingArm.setPosition(swingArmAngle);
    }


    public void scan() {
        // Swing arm to 60 degrees
        setSwingArmAngle(60);

        // Intake arm turret: Intaking position
        ArmTurr.setPosition(0.43);

        // Claw in vectoring position
        open();

        // Hand turret angle to be determined
        ClawTurr.setPosition(0.5); // Placeholder for TBD
    }

    public void hover() {
        // Swing arm angle at 15 degrees
        setSwingArmAngle(15);
    }

    public void pickup() {
        // Swing arm angle at -5 degrees
        setSwingArmAngle(-5);

        // Claw in latching position
        close();
    }

    public void transfer() {
        // Swing arm angle to 175 degrees
        setSwingArmAngle(175);

        // Hand turret to 90 degrees
        ArmTurr.setPosition(0.5); // 90 degrees assumed as 0.5
    }

    public void loiter() {
        // Claw in vectoring position
        open();
    }

}
