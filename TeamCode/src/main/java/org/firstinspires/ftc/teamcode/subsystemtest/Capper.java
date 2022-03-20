package org.firstinspires.ftc.teamcode.subsystemtest;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Capper implements Subsystem {

    CRServo capper;

    double capperPower = 0.6;

    @Override
    public void init(HardwareMap hardwareMap) {

        capper = hardwareMap.get(CRServo.class, "CAP");
        capper.setPower(0);
    }

    @Override
    public void run(Gamepad gamepad) {

    }
}
