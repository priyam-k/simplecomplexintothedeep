package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class Robot implements Subsystem{
    public Drivetrain drive;
    List<Subsystem> massInit;

    public Robot(){
        massInit = new ArrayList<>();
    }
    @Override
    public void init(HardwareMap hardwareMap) {
        //initilize here
        drive = new Drivetrain();

        //mass init
        massInit.add(drive);

//init here
        for(Subsystem s: massInit){
            s.init(hardwareMap);
        }

    }


    @Override
    public void update() {

    }

    @Override
    public void addTelemetry(Telemetry t) {

    }

}
