package org.firstinspires.ftc.teamcode.subsystemtest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Subsystem {
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    Gamepad gamepad2;
    public Subsystem(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;
    }
    abstract void init();
    abstract void run();
}
