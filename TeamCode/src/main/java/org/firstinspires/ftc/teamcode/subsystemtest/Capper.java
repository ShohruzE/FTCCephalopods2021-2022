package org.firstinspires.ftc.teamcode.subsystemtest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

public class Capper extends Subsystem {

    CRServo capper;
    CRServo capper2;

    public Capper(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void init() {

        hardwareMap = opMode.hardwareMap;
        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;

        capper = hardwareMap.get(CRServo.class, "CAP");
        capper2 = hardwareMap.get(CRServo.class, "CAP2");
    }

    @Override
    public void run() {

        if (gamepad1.a) {
            capper.setPower(0.5);
        }
        else if (gamepad1.y) {
            capper.setPower(-0.5);
        }
        else {
            capper.setPower(0);
        }


        if (gamepad1.left_bumper) {
            capper2.setPower(0.4);
        }
        else if (gamepad1.b) {
            capper2.setPower(-0.4);
        }
        else {
            capper2.setPower(0);
        }
    }
}
