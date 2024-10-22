package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Subsystem {

    void init(HardwareMap hardwareMap);

    void update();

    void addTelemetry(Telemetry t);
}