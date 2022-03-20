package org.firstinspires.ftc.teamcode.subsystemtest;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret implements Subsystem {

    DcMotor turret;

    double turretPower;

    @Override
    public void init(HardwareMap hardwareMap) {

        turret = hardwareMap.get(DcMotor.class, "TR");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void run(Gamepad gamepad) {

    }
}
