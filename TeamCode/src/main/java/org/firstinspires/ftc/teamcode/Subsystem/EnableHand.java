package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class EnableHand implements Subsystem{

    private Servo Claw,Larm,Rarm,ArmTurr,ClawTurr;


    @Override
    public void init(HardwareMap hardwareMap) {
        Claw = hardwareMap.get(Servo.class,"Servo2");
        Larm = hardwareMap.get(Servo.class,"Servo5");
        Rarm = hardwareMap.get(Servo.class,"Servo3");
        ArmTurr = hardwareMap.get(Servo.class,"Servo7");
        ClawTurr = hardwareMap.get(Servo.class,"Servo4");
    }

    @Override
    public void update() {

    }

    @Override
    public void addTelemtry(Telemetry t) {

    }

    private void BothArm(double x){
        Larm.setPosition(x);
        Rarm.setPosition(x);
    }

    pri
}
