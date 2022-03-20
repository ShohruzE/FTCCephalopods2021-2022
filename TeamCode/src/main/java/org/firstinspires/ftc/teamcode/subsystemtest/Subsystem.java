package org.firstinspires.ftc.teamcode.subsystemtest;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public interface Subsystem {
    void init(HardwareMap hardwareMap);
    void run(Gamepad gamepad);
}
