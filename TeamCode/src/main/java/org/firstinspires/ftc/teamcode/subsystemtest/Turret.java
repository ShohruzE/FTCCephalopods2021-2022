package org.firstinspires.ftc.teamcode.subsystemtest;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Turret extends Subsystem {


    DcMotor turret;
    RevTouchSensor leftTurretLimit;
    RevTouchSensor rightTurretLimit;
    RevTouchSensor middleTurretLimit;

    double turretSpeed = 0.35;

    DcMotor armMotor;
    RevTouchSensor lowArmHeightLimit;

    public Turret(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void init() {

       hardwareMap = opMode.hardwareMap;
       gamepad1 = opMode.gamepad1;
       gamepad2 = opMode.gamepad2;

        turret = hardwareMap.get(DcMotor.class, "TR");
        leftTurretLimit = hardwareMap.get(RevTouchSensor.class, "LTL");
        rightTurretLimit = hardwareMap.get(RevTouchSensor.class, "RTL");
        middleTurretLimit = hardwareMap.get(RevTouchSensor.class, "MTL");

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void run() {


        boolean leftSwitchIsPressed = leftTurretLimit.isPressed();
        boolean rightSwitchIsPressed = rightTurretLimit.isPressed();
        boolean middleSwitchIsPressed = middleTurretLimit.isPressed();

        /*
        if (middleSwitchIsPressed) {
            turret.setPower(0);
        }
         */

        if (gamepad2.x || gamepad1.dpad_left) {
            turret.setPower(-turretSpeed);

        }
        else if (gamepad2.b || gamepad1.dpad_right) {
            turret.setPower(turretSpeed);

        }
        else {
            turret.setPower(0);
        }
    }
}
