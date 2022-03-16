package org.firstinspires.ftc.teamcode.FTC_9385;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Shohruz on 10/10/21.
 */

@TeleOp(name = "TeleOp - Team_9385", group = "TeleOp")
@Disabled
public class TeleOp_9385 extends OpMode {

    // Motors
    DcMotor frontLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor backRightMotor = null;

    DcMotor arm = null;
    DcMotor carouselWheelLeft = null;
    DcMotor carouselWheelRight = null;
    CRServo intake;
    CRServo capperArm;

    // Motor Speed
    double frontLeftMotorSpeed;
    double frontRightMotorSpeed;
    double backLeftMotorSpeed;
    double backRightMotorSpeed;

    double intakeSpeed = 1.0;
    double armSpeed = 1.0;
    double carouselWheelSpeed = 0.8;

    private ElapsedTime runTime = new ElapsedTime();

    // Initialize all hardware that was mapped in Hardware_8087.java
    @Override
    public void init() {

        frontLeftMotor     = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor    = hardwareMap.get(DcMotor.class, "FR");
        backLeftMotor      = hardwareMap.get(DcMotor.class, "BL");
        backRightMotor     = hardwareMap.get(DcMotor.class, "BR");

        intake             = hardwareMap.get(CRServo.class, "Intake");
        arm                = hardwareMap.get(DcMotor.class, "Arm");
        carouselWheelLeft  = hardwareMap.get(DcMotor.class, "CWL");
        carouselWheelRight = hardwareMap.get(DcMotor.class, "CWR");
        capperArm          = hardwareMap.get(CRServo.class, "Cap");


        // Set motor direction
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(CRServo.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        carouselWheelLeft.setDirection(DcMotor.Direction.REVERSE);
        carouselWheelRight.setDirection(DcMotor.Direction.FORWARD);
        capperArm.setDirection(CRServo.Direction.REVERSE);


        // Set ZERO POWER BEHAVIOR
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselWheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselWheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.update();


    }

    @Override
    public void loop() {

        runTime.reset();

        drivetrain();
        intake();
        carouselWheel();
        moveArm();
        setCapperArm();

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
        vertical = -gamepad1.left_stick_y;
        spin = gamepad1.right_stick_x;

        frontLeftMotorSpeed = (-vertical - horizontal - spin) / denominator;
        frontRightMotorSpeed = (-vertical + horizontal + spin) / denominator;
        backLeftMotorSpeed = (vertical - horizontal + spin) / denominator;
        backRightMotorSpeed = (vertical + horizontal - spin) / denominator;

        frontLeftMotor.setPower(frontLeftMotorSpeed);
        frontRightMotor.setPower(frontRightMotorSpeed);
        backLeftMotor.setPower(backLeftMotorSpeed);
        backRightMotor.setPower(backRightMotorSpeed);
    }

    public void intake() {

        if (gamepad1.left_bumper) { // Intake

            intake.setPower(intakeSpeed);
        }
        else if (gamepad1.right_bumper) { // Reverse intake

            intake.setPower(-intakeSpeed);
        }
        else {
            intake.setPower(0);
        }
    }

    public void moveArm() {

        if (gamepad1.left_trigger > 0) { // Raise arm
            arm.setPower(armSpeed);
        }
        else if (gamepad1.right_trigger > 0) { // Lower arm
            arm.setPower(-0.5);
        }
        else { // Optional co-driver controls

            if (gamepad2.left_bumper) {
                arm.setPower(armSpeed);
            }
            else if (gamepad2.right_bumper) {
                arm.setPower(-0.5);
            }
            else {
                arm.setPower(0);
            }
        }
    }

    public void carouselWheel() {

        if (gamepad1.x) {

            carouselWheelLeft.setPower(-carouselWheelSpeed);
            carouselWheelRight.setPower(carouselWheelSpeed);
        }
        else if (gamepad1.b) {

            carouselWheelLeft.setPower(carouselWheelSpeed);
            carouselWheelRight.setPower(-carouselWheelSpeed);
        }
        else {
            carouselWheelLeft.setPower(0);
            carouselWheelRight.setPower(0);
        }
    }

    public void setCapperArm() {

        if (gamepad1.a) {
            capperArm.setPower(-1);
        }
        else if (gamepad1.y) {
            capperArm.setPower(1);
        }
        else { // Optional co-driver controls

            if (gamepad2.a) {
                capperArm.setPower(-1);
            }
            else if (gamepad2.y) {
                capperArm.setPower(1);
            }
            else {
                capperArm.setPower(0);
            }
        }
    }
}
