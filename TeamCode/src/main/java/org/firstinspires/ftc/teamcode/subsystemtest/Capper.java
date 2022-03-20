package org.firstinspires.ftc.teamcode.subsystemtest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Capper extends Subsystem {
    CRServo capper;

    public Capper(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void init() {
        capper = hardwareMap.get(CRServo.class, "CAP");
    }

    @Override
    public void run() {
        if (gamepad1.y) {
            capper.setPower(-0.5);
        }
        else if (gamepad1.a) {
            capper.setPower(0.5);
        }
        else {
            capper.setPower(0);
        }
    }
}
