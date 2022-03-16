package com.example.teamcode_8087

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Shohruz on 10/10/21.
 */

@TeleOp(name = "TeleOp - Team_8088", group = "TeleOp")
// @Disabled
public class TeleOp_8088 extends OpMode {

    // Motors
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    // Other Motors
    DcMotor intakeLeft;
    DcMotor cannonArm;


    // Motor Speed
    double frontLeftMotorSpeed;
    double frontRightMotorSpeed;
    double backLeftMotorSpeed;
    double backRightMotorSpeed;

    double intakeLeftMotorSpeed;
    double cannonArmMotorSpeed;

    private ElapsedTime runTime = new ElapsedTime();

    // Initialize all hardware that was mapped in Hardware_8088.java
    @Override
    public void init() {

        // Name all Motors
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "BL");
        backRightMotor  = hardwareMap.get(DcMotor.class, "BR");

        intakeLeft = hardwareMap.get(DcMotor.class, "IL");
        cannonArm = hardwareMap.get(DcMotor.class, "CA");
        // Servos


        // Set motor direction
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeLeft.setDirection(DcMotor.Direction.FORWARD);

        // Set ZERO POWER BEHAVIOR
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up encoders
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.update();


        runTime.reset();

    }

    @Override
    public void loop() {

        runTime.reset();

        drivetrain();

        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.addData("Motors", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), " +
                "backRight (%.2f)", frontLeftMotorSpeed, frontRightMotorSpeed, backLeftMotorSpeed, backRightMotorSpeed);
        telemetry.update();
    }

    @Override
    public void stop() {

        super.stop();
    }


    public void drivetrain() {


        double horizontal = 0; // x-axis movement
        double vertical = 0; // y-axis movement
        double spin = 0; // spin movement

        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(spin), 1);
        horizontal = gamepad1.left_stick_x * 1.1;
        vertical = - gamepad1.left_stick_y;
        spin = gamepad1.right_stick_x;

        frontLeftMotorSpeed = (vertical + horizontal + spin) / denominator;
        frontRightMotorSpeed = (vertical - horizontal - spin) / denominator;
        backLeftMotorSpeed = (vertical - horizontal + spin) / denominator;
        backRightMotorSpeed = (vertical + horizontal - spin) / denominator;

        frontLeftMotor.setPower(frontLeftMotorSpeed);
        frontRightMotor.setPower(frontRightMotorSpeed);
        backLeftMotor.setPower(backLeftMotorSpeed);
        backRightMotor.setPower(backRightMotorSpeed);
    }
}
