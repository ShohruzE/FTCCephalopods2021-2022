package org.firstinspires.ftc.teamcode.subsystemtest;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Turret extends Subsystem {

    DcMotor turret;
    RevTouchSensor leftTurretLimit;
    RevTouchSensor rightTurretLimit;

    double turretSpeed = .6;

    public Turret(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void init() {

       hardwareMap = opMode.hardwareMap;

        turret = hardwareMap.get(DcMotor.class, "TR");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftTurretLimit = hardwareMap.get(RevTouchSensor.class, "LTL");
        rightTurretLimit = hardwareMap.get(RevTouchSensor.class, "RTL");
    }

    @Override
    public void run() {

        boolean leftSwitchIsPressed = leftTurretLimit.isPressed();
        boolean rightSwitchIsPressed = rightTurretLimit.isPressed();

        if (gamepad2.x) {
            turret.setPower(-turretSpeed);

            if (leftSwitchIsPressed) {
                turret.setPower(0);
            }
        }
        else if (gamepad2.b) {
            turret.setPower(turretSpeed);

            if (rightSwitchIsPressed) {
                turret.setPower(0);
            }
        }
        else {
            if (gamepad1.dpad_left) {
                turret.setPower(-turretSpeed);

                if (leftSwitchIsPressed) {
                    turret.setPower(0);
                }
            }
            else if (gamepad1.dpad_right) {
                turret.setPower(turretSpeed);

                if (rightSwitchIsPressed) {
                    turret.setPower(0);
                }
            }
            else {
                turret.setPower(0);
            }
        }
    }
}
