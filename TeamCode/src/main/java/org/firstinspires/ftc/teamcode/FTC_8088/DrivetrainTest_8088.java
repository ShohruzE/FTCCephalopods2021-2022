package org.firstinspires.ftc.teamcode.FTC_8088;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Drivetrain Test", group = "TeleOp Test - 8088")
@Disabled
public class DrivetrainTest_8088 extends OpMode {

    // Motors
    DcMotor frontLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor backRightMotor = null;

    // Motor Speed
    double frontLeftMotorSpeed;
    double frontRightMotorSpeed;
    double backLeftMotorSpeed;
    double backRightMotorSpeed;

    double powerMultiplier = 1.0;

    private ElapsedTime runTime = new ElapsedTime();

    @Override
    public void init() {

        // Name all Motors
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "BL");
        backRightMotor  = hardwareMap.get(DcMotor.class, "BR");

        // Set motor direction
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set ZERO POWER BEHAVIOR
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.update();

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

        if (gamepad1.left_bumper) {
            powerMultiplier = 0.4;
        }
        else {
            powerMultiplier = 1.0;
        }

        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(spin), 1);
        horizontal = gamepad1.left_stick_x * 1.1;
        vertical = - gamepad1.left_stick_y;
        spin = gamepad1.right_stick_x;

        frontLeftMotorSpeed = (vertical + horizontal + spin) / denominator;
        frontRightMotorSpeed = (vertical - horizontal - spin) / denominator;
        backLeftMotorSpeed = (vertical - horizontal + spin) / denominator;
        backRightMotorSpeed = (vertical + horizontal - spin) / denominator;

        frontLeftMotor.setPower(frontLeftMotorSpeed * powerMultiplier);
        frontRightMotor.setPower(frontRightMotorSpeed * powerMultiplier);
        backLeftMotor.setPower(backLeftMotorSpeed * powerMultiplier);
        backRightMotor.setPower(backRightMotorSpeed * powerMultiplier);
    }
}
