package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class Robot implements Subsystem {
    public Drivetrain drive;
    List<Subsystem> massInit;

    public Robot() {
        massInit = new ArrayList<>();
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        drive = new Drivetrain();

        massInit.add(drive);
        for (Subsystem s : massInit) {
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
