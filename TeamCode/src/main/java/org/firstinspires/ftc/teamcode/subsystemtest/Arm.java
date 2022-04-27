package org.firstinspires.ftc.teamcode.subsystemtest;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm extends Subsystem {

    DcMotor armMotor;
    RevTouchSensor lowArmHeightLimit;

    double armMotorSpeed = 0.45;

    public Arm(OpMode opMode) {
        super(opMode);
    }


    @Override
    public void init() {

        hardwareMap = opMode.hardwareMap;
        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;

        armMotor = hardwareMap.get(DcMotor.class, "ARM");
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowArmHeightLimit = hardwareMap.get(RevTouchSensor.class, "LAHL");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void run() {


        boolean lowArmHeightLimitReached = lowArmHeightLimit.isPressed();

        if (lowArmHeightLimitReached) {
            armMotor.setPower(0);
        }
        else {
            // Raises arm
            if (gamepad2.dpad_up) {
                armMotor.setPower(0.5);
            }
            // Lowers arm
            else if (gamepad2.dpad_down) {
                armMotor.setPower(-0.4);
            }
            else {

                if (gamepad1.dpad_down) {
                    armMotor.setPower(-0.4);
                }
                else if (gamepad1.dpad_up) {
                    armMotor.setPower(0.3);
                }
                else {
                    armMotor.setPower(0);
                }
            }
        }

        /*

        if (gamepad2.dpad_up) {
            armMotor.setPower(0.5);
        }
        // Lowers arm
        else if (gamepad2.dpad_down) {
            armMotor.setPower(-0.35);
        }
        else {

            if (gamepad1.a) {
                armMotor.setPower(-0.35);
            } else if (gamepad1.y) {
                armMotor.setPower(0.5);
            } else {
                armMotor.setPower(0);
            }
        }

        */

    }
}
