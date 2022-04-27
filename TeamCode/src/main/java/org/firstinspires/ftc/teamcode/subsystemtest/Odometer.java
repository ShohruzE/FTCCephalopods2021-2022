package org.firstinspires.ftc.teamcode.subsystemtest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Odometer extends Subsystem {

    Servo odometerRetract;

    public Odometer(OpMode opMode) {
        super(opMode);
    }

    @Override
    void init() {

        odometerRetract = hardwareMap.get(Servo.class, "ODO");
        odometerRetract.setPosition(0.45);
    }

    @Override
    void run() {


    }
}
