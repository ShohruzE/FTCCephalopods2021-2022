package org.firstinspires.ftc.teamcode.subsystemtest;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm implements Subsystem {

    DcMotor arm;

    double armPower = 0.45;

    @Override
    public void init(HardwareMap hardwareMap) {

        arm = hardwareMap.get(DcMotor.class, "ARM");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void run(Gamepad gamepad) {

    }
}
