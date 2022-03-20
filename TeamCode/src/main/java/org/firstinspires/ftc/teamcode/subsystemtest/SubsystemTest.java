package org.firstinspires.ftc.teamcode.subsystemtest;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubsystemTest implements Subsystem {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    double frontLeftMotorPower;
    double frontRightMotorPower;
    double backLeftMotorPower;
    double backRightMotorPower;

    @Override
    public void init(HardwareMap hardwareMap) {

        frontLeft   = hardwareMap.get(DcMotor.class, "FL");
        frontRight  = hardwareMap.get(DcMotor.class, "FR");
        backLeft    = hardwareMap.get(DcMotor.class, "BL");
        backRight   = hardwareMap.get(DcMotor.class, "BR");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    @Override
    public void run(Gamepad gamepad) {

        double horizontal = 0; // x-axis movement
        double vertical = 0; // y-axis movement
        double spin = 0; // rotational movement

        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(spin), 1);
        horizontal = gamepad.left_stick_x;
        vertical = -gamepad.left_stick_y;
        spin = gamepad.right_stick_x;

        frontLeftMotorPower = (vertical + horizontal + spin) / denominator;
        frontRightMotorPower = (vertical - horizontal - spin) / denominator;
        backLeftMotorPower = (vertical - horizontal + spin) / denominator;
        backRightMotorPower = (vertical + horizontal - spin) / denominator;

        frontLeft.setPower(frontLeftMotorPower);
        frontRight.setPower(frontRightMotorPower);
        backLeft.setPower(backLeftMotorPower);
        backRight.setPower(backRightMotorPower);
    }
}