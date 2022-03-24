package org.firstinspires.ftc.teamcode.subsystemtest;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Drive extends Subsystem {


    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    /*
    Servo odometerVerticalLeft;
    Servo odometerVerticalRight;
    Servo odometerLateral;
    */

    double frontLeftMotorPower;
    double frontRightMotorPower;
    double backLeftMotorPower;
    double backRightMotorPower;

    public Drive(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void init() {

        hardwareMap = opMode.hardwareMap;
        gamepad1 = opMode.gamepad1;

        frontLeft   = hardwareMap.get(DcMotor.class, "FL");
        frontRight  = hardwareMap.get(DcMotor.class, "FR");
        backLeft    = hardwareMap.get(DcMotor.class, "BL");
        backRight   = hardwareMap.get(DcMotor.class, "BR");

        /*
        odometerVerticalLeft = hardwareMap.get(Servo.class, "OVL");
        odometerVerticalRight = hardwareMap.get(Servo.class, "OVR");
        odometerLateral = hardwareMap.get(Servo.class, "OL");
        */

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

    @Override
    public void run() {

        double horizontal = 0; // x-axis movement
        double vertical = 0; // y-axis movement
        double spin = 0; // rotational movement
        double powerMultiplier;

        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(spin), 1);
        horizontal = gamepad1.left_stick_x;
        vertical = -gamepad1.left_stick_y;
        spin = gamepad1.right_stick_x;

        if (gamepad1.left_bumper) {
            powerMultiplier = 0.4;
        }
        else {
            powerMultiplier = 1;
        }

        frontLeftMotorPower = (vertical + horizontal + spin) / denominator;
        frontRightMotorPower = (vertical - horizontal - spin) / denominator;
        backLeftMotorPower = (vertical - horizontal + spin) / denominator;
        backRightMotorPower = (vertical + horizontal - spin) / denominator;

        frontLeft.setPower(frontLeftMotorPower * powerMultiplier);
        frontRight.setPower(frontRightMotorPower * powerMultiplier);
        backLeft.setPower(backLeftMotorPower * powerMultiplier);
        backRight.setPower(backRightMotorPower * powerMultiplier);
    }
}
