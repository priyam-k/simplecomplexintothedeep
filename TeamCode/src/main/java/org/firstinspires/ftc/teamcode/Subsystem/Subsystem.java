package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Subsystem {
    void init(HardwareMap map, Gamepad g1, Gamepad g2);
    void update();
    void addTelemtry(Telemetry t);
}