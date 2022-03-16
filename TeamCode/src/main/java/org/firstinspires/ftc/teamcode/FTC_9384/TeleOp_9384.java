package org.firstinspires.ftc.teamcode.FTC_9384;

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

@TeleOp(name = "TeleOp - Team_9384", group = "TeleOp")
@Disabled
public class TeleOp_9384 extends OpMode {

    // Motors
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    CRServo intake;
    DcMotor elevator;
    Servo bucket;
    DcMotor carouselWheel;

    public final static double BUCKET_START = 1;
    public final static double BUCKET_DROP3 = 0.5;
    public final static double BUCKET_DROP1 = -1;


    // Motor Speed
    double frontLeftMotorSpeed;
    double frontRightMotorSpeed;
    double backLeftMotorSpeed;
    double backRightMotorSpeed;

    double elevatorSpeed = 0.8;
    double intakeSpeed = 1;
    double carouselWheelSpeed = 1;

    double powerMultiplier = 1.0;

    private ElapsedTime runTime = new ElapsedTime();

    // Initialize all hardware that was mapped in Hardware_8087.java
    @Override
    public void init() {

        frontLeftMotor  = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "BL");
        backRightMotor  = hardwareMap.get(DcMotor.class, "BR");


        intake          = hardwareMap.get(CRServo.class, "Intake");
        elevator        = hardwareMap.get(DcMotor.class, "EL");
        bucket          = hardwareMap.get(Servo.class, "B");
        carouselWheel   = hardwareMap.get(DcMotor.class, "CW");


        // Set motor direction
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);


        intake.setDirection(CRServo.Direction.REVERSE);
        elevator.setDirection(DcMotor.Direction.REVERSE);
        carouselWheel.setDirection(DcMotor.Direction.FORWARD);


        // Set ZERO POWER BEHAVIOR
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bucket.setPosition(BUCKET_START);

        // Set up encoders
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.update();


    }

    @Override
    public void loop() {

        runTime.reset();

        drivetrain();
        intake();
        setElevator();
        setBucket();
        carouselWheel();

        setElevatorEncoder();

        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.addData("Motors", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), " +
                "backRight (%.2f)", frontLeftMotorSpeed, frontRightMotorSpeed, backLeftMotorSpeed, backRightMotorSpeed);
        telemetry.addData("Elevator: ", elevator.getCurrentPosition());
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
            powerMultiplier = 0.5;
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

    public void carouselWheel() {

        if (gamepad1.x) {

            carouselWheel.setDirection(DcMotor.Direction.FORWARD);
            carouselWheel.setPower(carouselWheelSpeed);
        }
        else if (gamepad1.b) {

            carouselWheel.setDirection(DcMotor.Direction.REVERSE);
            carouselWheel.setPower(carouselWheelSpeed);
        }
        else {
            carouselWheel.setPower(0);
        }
    }

    public void intake() {

        if (gamepad1.right_bumper) {
            intake.setPower(intakeSpeed);
        }

        else if (gamepad1.left_bumper) {
            intake.setPower(-intakeSpeed);
        }
        else {
            intake.setPower(0);
        }

    }

    public void setElevator() {

        if (gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0) {
            elevator.setPower(elevatorSpeed);
        }
        else if (gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0) {
            elevator.setPower(-elevatorSpeed);
        }
        else {
            elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void setBucket() {

        if (gamepad1.y || gamepad2.y) {
            bucket.setPosition(BUCKET_DROP3);
        }
        else if (gamepad1.a || gamepad2.a) {
            bucket.setPosition(BUCKET_START);
        }
    }

    public void setElevatorEncoder() {

        if (gamepad1.dpad_up) {
            elevator.setTargetPosition(100);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setPower(0.4);
        }
        else if (gamepad1.dpad_down) {
            elevator.setTargetPosition(-100);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setPower(0.4);
        }
    }
}
