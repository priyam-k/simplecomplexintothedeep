package org.firstinspires.ftc.teamcode.Subsystem;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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

    public double IntakeTurretAngleAutoAlign(double Pixelerror,double PixelBound,double Increment){
        double increment = 0;
        if (Math.abs(Pixelerror)> PixelBound){
           increment = (Pixelerror > 0)? Increment : -Increment;
            ArmTurr.setPosition(ArmTurr.getPosition() + increment);
        }
        return 306.12245*ArmTurr.getPosition() - 133.77551;
    }

    public static double offset = 186;

    // only do degree values between -90 and 180 otherwise it will explode
    public double setHandTurretDegrees(double degre){
        return (((degre-offset)/270.0)+3)%1;

    }
    //bro

    private double degreesToTicks(double d) {
        return d / 270;
    }
    private double axonDegreesToPos(double d) { // generated from python script, works for intake turret
        return 0.003076923076923078*(d - 87.1) + 0.43;
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
    public void changeArmTurrAngle(double da) {
        ArmTurr.setPosition(ArmTurr.getPosition() + axonDegreesToPos(da));
    }

    public void setSwingArmAngle(double angle) {
        double swingArmAngle = degreesToTicksSwingArm(angle);
        LServoSwingArm.setPosition(swingArmAngle);
        RServoSwingArm.setPosition(swingArmAngle);
        ArmTurr.setPosition(0.435);
    }

    public void setSwingAngleOnlyAngle(double angle){
        double swingArmAngle = degreesToTicksSwingArm(angle);
        LServoSwingArm.setPosition(swingArmAngle);
        RServoSwingArm.setPosition(swingArmAngle);

    }
    public void setSwingArmAngleAuton(double angle) {
        double swingArmAngle = degreesToTicksSwingArm(angle);
        LServoSwingArm.setPosition(swingArmAngle);
        RServoSwingArm.setPosition(swingArmAngle);
        ArmTurr.setPosition(0.45); //to account for camera not being staight (0.45)
        //now we Account for it in vision code
    }
    public void setSwingArmAngleAdiRunner(double angle,double armturr){
        double swingArmAngle = degreesToTicksSwingArm(angle);
        LServoSwingArm.setPosition(swingArmAngle);
        RServoSwingArm.setPosition(swingArmAngle);
        ArmTurr.setPosition(armturr);
    }





    public void scan1() {
        // Swing arm to 60 degrees
        setSwingArmAngle(60);
    }

    public void scan2() {
        // Intake arm turret: Intaking position
        ArmTurr.setPosition(0.43);
    }

    public void scan3() {
        // Claw in vectoring position
        open();
    }

    public void scan4() {
        // Hand turret angle to 90 degrees (TBD)
        ClawTurr.setPosition(setHandTurretDegrees(90)); // Placeholder for TBD
    }

    public void hover1(){
        setSwingArmAngle(15);// Swing arm angle at 15 degrees
    }
    public void hover2() {

        if (gamepad.dpad_left) {
            wasPressed = true;
        }
        if (gamepad.dpad_right) {
            wasPressedR = true;
        }
        if(gamepad.dpad_up){
            waspressedGP = true;
        }
        if(!gamepad.dpad_up && waspressedGP){
            ClawTurr.setPosition(setHandTurretDegrees(0));
            waspressedGP = false;
        }
         if(!gamepad.dpad_left && wasPressed){
            ClawTurr.setPosition(ClawTurr.getPosition()+0.05);
            wasPressed = false;
        }
        if(!gamepad.dpad_right && wasPressedR){
            ClawTurr.setPosition(ClawTurr.getPosition()-0.05);
            wasPressedR = false;
        }
    }

    public void hoverAuto(){
        setSwingArmAngle(30);
    }

    public void autonPickup1(){ClawTurr.setPosition(setHandTurretDegrees(90));}
    public void pickup1() {
        // Swing arm angle at -5 degrees
        setSwingArmAngle(-5);
    }
    public void donthitsampleplease(){
        setSwingArmAngle(75);
    }

    public void pickup2Auton(){
        setSwingArmAngle(5);
    }


    public void pickup2() {
        // Claw in latching position
        close();
    }

    public void transfer1() {
        // NOT BEING USED
        // Delete all usages
        setSwingArmAngle(100);
    }
    public void turrSet0Auton(){
        //Set turret position to 0
        ArmTurr.setPosition(0.44);
    }

    public void transfer1point5() {
        setSwingArmAngle(180);
    }


    public void transfer2() {
        // Hand turret to 0 degrees
        ClawTurr.setPosition(setHandTurretDegrees(0));
    }
    public void autonTransfer2(){ClawTurr.setPosition(setHandTurretDegrees(90));}

    public void intaketrasnferinone(){
        transfer1point5();
        turrSet0Auton();
        autonTransfer2();
    }

    public void loiter() {
        // Claw in vectoring position
        open();
    }


}
