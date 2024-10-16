package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class EnableHand implements Subsystem{

    private Servo Claw, LServoSwingArm, RServoSwingArm,ArmTurr,ClawTurr;
    private int ServoAngleofAxon = 255;


    @Override
    public void init(HardwareMap hardwareMap) {
        Claw = hardwareMap.get(Servo.class,"Servo2");
        LServoSwingArm = hardwareMap.get(Servo.class,"Servo7"); // left servo for intake swing arm
        RServoSwingArm = hardwareMap.get(Servo.class,"Servo8"); // right servo for intake swing arm
        ArmTurr = hardwareMap.get(Servo.class,"Servo1");
        ClawTurr = hardwareMap.get(Servo.class,"Servo4");
    }

    @Override
    public void update() {

    }

    @Override
    public void addTelemtry(Telemetry t) {

    }


    public void BothArm(double x){
        LServoSwingArm.setPosition(1-x);
        RServoSwingArm.setPosition(x);
    }

    private double degreesToTicksAxon(double d){return d/ServoAngleofAxon;}
    private double degreesToTicks(double d){return d/270;}

    public void close(){Claw.setPosition(0.63);}

    public void open(){Claw.setPosition(0.44);}


    void setSwingArmAngle(double angle){
        double swingArmAngle = degreesToTicks(angle);
        LServoSwingArm.setPosition(swingArmAngle);
        RServoSwingArm.setPosition(swingArmAngle);
    }

    public void scan() {
        setSwingArmAngle(60);
    }

    public void hover() {
        setSwingArmAngle(10);
    }

    public void pickup() {
        setSwingArmAngle(0);
    }

    public void transfer() {
        setSwingArmAngle(170);
    }

    public void loiter() {
        setSwingArmAngle(170);
        // open claw to release sample to outtake
    }

}
