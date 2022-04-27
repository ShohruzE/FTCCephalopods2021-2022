package org.firstinspires.ftc.teamcode.subsystemtest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class CarouselWheel extends Subsystem {


    DcMotorEx carouselWheelLeft;
    DcMotorEx carouselWheelRight;

    double carouselWheelSpeed = 0.4;

    public CarouselWheel(OpMode opMode) {
        super(opMode);
    }

    @Override
    void init() {

        hardwareMap = opMode.hardwareMap;
        gamepad1 = opMode.gamepad1;

        carouselWheelLeft = hardwareMap.get(DcMotorEx.class, "CWL");
        carouselWheelRight = hardwareMap.get(DcMotorEx.class, "CWR");

        carouselWheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselWheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carouselWheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselWheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        carouselWheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselWheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    void run() {

        if (gamepad1.x) {

            carouselWheelLeft.setPower(0.8);
            carouselWheelRight.setPower(-0.8);

            /*
            if (carouselWheelLeft.getCurrentPosition() >= 1000 || carouselWheelRight.getCurrentPosition() >= -1000) {
                carouselWheelLeft.setPower(0.7);
                carouselWheelRight.setPower(-0.7);

                if (carouselWheelLeft.getCurrentPosition() >= 5000 || carouselWheelRight.getCurrentPosition() >= -5000) {
                    carouselWheelLeft.setPower(1);
                    carouselWheelRight.setPower(-1);
                }
            }

             */
        }
        else {
            /*
            carouselWheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            carouselWheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             */
            carouselWheelLeft.setPower(0);
            carouselWheelRight.setPower(0);
        }
    }
}
