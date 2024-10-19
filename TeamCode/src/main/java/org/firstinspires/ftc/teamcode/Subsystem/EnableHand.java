package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class EnableHand implements Subsystem{

    private Servo Claw, LServoSwingArm, RServoSwingArm,ArmTurr,ClawTurr;

/*
Latching on to the sample: 0.43
Vetoring position: 0.6
 */
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

    private double degreesToTicks(double d){return d/270;}

    public void close(){Claw.setPosition(0.63);}

    public void open(){Claw.setPosition(0.44);}


    void setSwingArmAngle(double angle){
        double swingArmAngle = degreesToTicks(angle);
        LServoSwingArm.setPosition(swingArmAngle);
        RServoSwingArm.setPosition(swingArmAngle);
    }

    /*
    Intake arm turret
    Intaking: 0.43

     */

    /*
    Claw
    Latching: 0.43
    vectoring: 0.63
     */


    public void scan() {

        //swing arm 60 degrees
        //intake arm turret --> Intaking
        //Claw: Vectoring
        //Hand turretdegree: TBD

    }

    public void hover() {
       //Swign arm angle at 15

    }

    public void pickup() {

        //swing arm angle: -5
        //Claw: latching

    }

    public void transfer() {
        //swing arm angle : 175
        //Hand turret 90 degrees
    }
    public void loiter() {
        //Claw: Vector
    }

}
