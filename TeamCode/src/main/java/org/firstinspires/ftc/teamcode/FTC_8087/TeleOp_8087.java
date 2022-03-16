package org.firstinspires.ftc.teamcode.FTC_8087;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Shohruz on 10/10/21.
 */

@TeleOp(name = "TeleOp - Team_8087", group = "TeleOp")
@Disabled
public class TeleOp_8087 extends OpMode {

    // Motors
    DcMotor frontLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor backRightMotor = null;

    // Other Motors
    DcMotor intake = null;
    DcMotor elevator = null;
    DcMotor carouselWheelRight = null;
    DcMotor carouselWheelLeft = null;

    Servo bucket;
    Servo bucketLock;
    CRServo tapeMeasure;
    CRServo tapeMeasureTurret;
    CRServo tapeMeasureAngle;

    // Motor Speed
    double frontLeftMotorSpeed;
    double frontRightMotorSpeed;
    double backLeftMotorSpeed;
    double backRightMotorSpeed;

    double intakeSpeed = 1.0;
    double elevatorSpeed = 0.4;
    double carouselWheelSpeed = 1.0;
    double tapeMeasureSpeed = 1;

    public final static double BUCKET_START = 0.0;
    public final static double BUCKET_DROP = 0.6;

    double powerMultiplier = 1.0;

    private ElapsedTime runTime = new ElapsedTime();

    @Override
    public void init() {

        // Name all Motors
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "BL");
        backRightMotor  = hardwareMap.get(DcMotor.class, "BR");

        intake          = hardwareMap.get(DcMotor.class, "Intake");
        elevator        = hardwareMap.get(DcMotor.class, "EL");
        carouselWheelRight   = hardwareMap.get(DcMotor.class, "CWR");
        carouselWheelLeft   = hardwareMap.get(DcMotor.class, "CWL");

        bucket          = hardwareMap.get(Servo.class, "BU");
        tapeMeasure     = hardwareMap.get(CRServo.class, "TP");
        tapeMeasureTurret =  hardwareMap.get(CRServo.class, "TPT");
        tapeMeasureAngle = hardwareMap.get(CRServo.class, "TPA");


        // Set motor direction
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.FORWARD);


        // Set ZERO POWER BEHAVIOR
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bucket.setPosition(BUCKET_START);
        bucketLock.setPosition(BUCKET_START);


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.update();


    }

    @Override
    public void loop() {

        runTime.reset();

        drivetrain();
        setElevator();
        setBucket();
        setBucketLock();
        intake();
        // carouselWheel();
        setTapeMeasure();
        setTapeMeasureTurret();
        setTapeMeasureAngle();


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

        if (gamepad1.left_bumper || gamepad1.right_bumper) {
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

    public void setElevator() {

        if (gamepad1.left_trigger > 0) {

            elevator.setPower(elevatorSpeed);
        }
        else if (gamepad1.right_trigger > 0) {

            elevator.setPower(-elevatorSpeed);
        }
        else {
            elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void setBucket() {

        if (gamepad1.a) {
            bucket.setPosition(BUCKET_DROP);
        }
        else if (gamepad1.y) {
            bucket.setPosition(BUCKET_START);
        }
    }

    public void setBucketLock() {

        if (gamepad2.a) {
            bucket.setPosition(0.5);
        }
        else if (gamepad2.b) {
            bucket.setPosition(0.0);
        }
    }

    public void intake() {

        if (gamepad1.left_bumper) {

            intake.setPower(intakeSpeed);
        }
        else if (gamepad1.right_bumper) {

            intake.setPower(-intakeSpeed);
        }
        else {
            intake.setPower(0);
        }
    }


    public void carouselWheel() {

        if (gamepad1.x) {

            carouselWheelLeft.setPower(carouselWheelSpeed);
            carouselWheelRight.setPower(carouselWheelSpeed);
        }
        else if (gamepad1.b) {

            carouselWheelLeft.setPower(-carouselWheelSpeed);
            carouselWheelRight.setPower(-carouselWheelSpeed);
        }
        else {
            carouselWheelLeft.setPower(0);
        }
    }

    public void setTapeMeasure() {

        if (gamepad2.left_bumper) {
            tapeMeasure.setPower(tapeMeasureSpeed);
        }
        else if (gamepad2.right_bumper) {
            tapeMeasure.setPower(-tapeMeasureSpeed);
        }
        else {
            tapeMeasure.setPower(0);
        }
    }

    public void setTapeMeasureTurret() {

        if (gamepad2.right_trigger > 0) {
            tapeMeasureTurret.setPower(0.2);
        }
        else if (gamepad2.left_trigger > 0) {
            tapeMeasureTurret.setPower(-0.2);
        }
        else {
            tapeMeasureTurret.setPower(0);
        }
    }

    public void setTapeMeasureAngle() {

        if (gamepad2.dpad_up) {
            tapeMeasureAngle.setPower(0.2);
        }
        else if (gamepad2.dpad_down) {
            tapeMeasureAngle.setPower(-0.2);
        }
    }
}


