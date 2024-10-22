package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake implements Subsystem {

    Servo turret;
    Servo handTurret;
    Servo rightSwing;
    Servo leftSwing;
    Servo intakeClaw;

    @Override
    public void init(HardwareMap hardwareMap) {
        // Initialize the Servos
        turret = hardwareMap.servo.get("Servo7");
        handTurret = hardwareMap.servo.get("Servo4");
        rightSwing = hardwareMap.servo.get("Servo3");
        leftSwing = hardwareMap.servo.get("Servo5");
        intakeClaw = hardwareMap.servo.get("Servo2");
    }



    public void idle() {

    }

    public void drop() {

    }


    public void grab() {

    }


    public void flip() {

    }


    @Override
    public void update() {

    }

    @Override
    public void addTelemetry(Telemetry t) {

    }

}
