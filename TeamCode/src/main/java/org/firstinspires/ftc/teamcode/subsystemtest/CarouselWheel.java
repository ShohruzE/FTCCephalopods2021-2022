package org.firstinspires.ftc.teamcode.subsystemtest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class CarouselWheel extends Subsystem {


    DcMotorEx carouselWheelLeft;
    DcMotorEx carouselWheelRight;

    double carouselWheelSpeed = 0.6;

    public CarouselWheel(OpMode opMode) {
        super(opMode);
    }

    @Override
    void init() {

        hardwareMap = opMode.hardwareMap;
        gamepad1 = opMode.gamepad1;

        carouselWheelLeft = hardwareMap.get(DcMotorEx.class, "CWL");
        carouselWheelRight = hardwareMap.get(DcMotorEx.class, "CWR");

        carouselWheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselWheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    void run() {

        if (gamepad1.x) {

            carouselWheelLeft.setPower(carouselWheelSpeed);
            carouselWheelRight.setPower(-carouselWheelSpeed);
        }
        else if (gamepad1.b) {

            carouselWheelLeft.setPower(-carouselWheelSpeed);
            carouselWheelRight.setPower(carouselWheelSpeed);
        }
        else {
            carouselWheelLeft.setPower(0);
            carouselWheelRight.setPower(0);
        }
    }
}
