package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake implements Subsystem {

    /* Constants */

    // Arm positions: ground, middle (idle), top (transfer),

    public boolean check = false;
    Servo turret, handTurret, rightSwing, leftSwing, intakeClaw, wrist, rightArm, leftArm, outtakeClaw;
    private Outtake.OuttakeStates os;

    @Override
    public void init(HardwareMap hardwareMap) {
        turret = hardwareMap.servo.get("Servo7");
        handTurret = hardwareMap.servo.get("Servo4");
        rightSwing = hardwareMap.servo.get("Servo3");
        leftSwing = hardwareMap.servo.get("Servo5");
        intakeClaw = hardwareMap.servo.get("Servo2");
        //wrist = hardwareMap.servo.get("wrist");
        //rightArm = hardwareMap.servo.get("rightArm");
        //leftArm = hardwareMap.servo.get("leftArm");
        //outtakeClaw = hardwareMap.servo.get("ottakeClaw");

        // Reset motor power
    }

    public void LoiteringPosition() throws InterruptedException {

    }

    public void loiter1() {
        turret.setPosition(0.43);
        Swing(0.7);
        handTurret.setPosition(0);
        intakeClaw.setPosition(0.63);
    }

    public void grab1() {
        Swing(0.9);
    }

    public void finalGrab() {
        intakeClaw.setPosition(0.44);
    }
    public void back1() {
        Swing(0.52);
        turret.setPosition(0.43);
        handTurret.setPosition(0);
    }

    @Override
    public void update() {
    }

    @Override
    public void addTelemtry(Telemetry t) {

    }




    public Outtake.OuttakeStates getOTState() {
        return os;
    }

    public void setOTState(Outtake.OuttakeStates state) {
        os = state;
    }

    public void Swing(double d) {
        rightSwing.setPosition(d);
        leftSwing.setPosition(d);
    }

    public void Arm(double d){
        rightArm.setPosition(d);
        leftArm.setPosition(d);
    }

    public void incrementTurretServo(double d){
        turret.setPosition(d);
    }
    public void incrementHandTurret(double d){
        handTurret.setPosition(d);
    }




    public enum OuttakeStates {
        LOITER1,
        GRAB1, ROTATEPOS, FINALGRAB,
        BACK1,
    }


}
