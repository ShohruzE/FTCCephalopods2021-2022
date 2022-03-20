package org.firstinspires.ftc.teamcode.subsystemtest;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake implements Subsystem {

    CRServo intake;


    double intakePower;

    @Override
    public void init(HardwareMap hardwareMap) {

        intake = hardwareMap.get(CRServo.class, "Intake");
        intake.setPower(0);
    }

    @Override
    public void run(Gamepad gamepad) {

    }
}
