package org.firstinspires.ftc.teamcode.FTC_8088;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

    DcMotor armMotor;
    DcMotor turret;
    DcMotor carouselWheelLeft;
    DcMotor carouselWheelRight;

    CRServo intake;
    CRServo capper;

    RevTouchSensor leftTurretLimit;
    RevTouchSensor rightTurretLimit;
    RevTouchSensor maxArmHeightLimit;
    RevColorSensorV3 colorSensor;
    ColorSensor colorSensor2;


    // Motor Speed
    double frontLeftMotorSpeed;
    double frontRightMotorSpeed;
    double backLeftMotorSpeed;
    double backRightMotorSpeed;

    double intakeSpeed = 1;
    double turretSpeed = 0.6;
    double armMotorSpeed = 0.45;
    double carouselWheelSpeed = 0.65;

    static final double INCREMENT   = 0.05;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   250;     // period of each cycle
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor

    boolean rampUp = true;


    double powerMultiplier = 1.0;

    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime carouselTimer = new ElapsedTime();


    // Initialize all hardware that was mapped in Hardware_8087.java
    @Override
    public void init() {

        frontLeftMotor   = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor  = hardwareMap.get(DcMotor.class, "FR");
        backLeftMotor    = hardwareMap.get(DcMotor.class, "BL");
        backRightMotor   = hardwareMap.get(DcMotor.class, "BR");

        turret           = hardwareMap.get(DcMotor.class, "TR");
        armMotor         = hardwareMap.get(DcMotor.class, "ARM");
        carouselWheelLeft    = hardwareMap.get(DcMotor.class, "CWL");
        carouselWheelRight = hardwareMap.get(DcMotor.class, "CWR");

        capper           = hardwareMap.get(CRServo.class, "CAP");
        intake           = hardwareMap.get(CRServo.class, "Intake");

        colorSensor      = hardwareMap.get(RevColorSensorV3.class, "CS");
        colorSensor2     = hardwareMap.get(ColorSensor.class, "CS2");
        leftTurretLimit = hardwareMap.get(RevTouchSensor.class, "LTL");
        rightTurretLimit = hardwareMap.get(RevTouchSensor.class, "RTL");
        maxArmHeightLimit = hardwareMap.get(RevTouchSensor.class, "MAHL");



        // Set motor direction
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);


        // Set ZERO POWER BEHAVIOR
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselWheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselWheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        capper.setPower(0);

        // Set up encoders
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.update();

        runTime.reset();
    }

    @Override
    public void loop() {


        drivetrain();
        intake();
        setArm();
        setTurret();
        carouselWheel();

        setCapper();
        rumble();

      //  colorSensor();

        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.addData("Motors", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), " +
                "backRight (%.2f)", frontLeftMotorSpeed, frontRightMotorSpeed, backLeftMotorSpeed, backRightMotorSpeed);
        telemetry.addData("Turret: ", turret.getCurrentPosition());
        telemetry.addData("Arm: ", armMotor.getCurrentPosition());

        telemetry.addData("Left Switch,", leftSwitchIsPressed());
        telemetry.addData("Right Switch,", rightSwitchIsPressed());
        telemetry.addData("Max arm Height,", touchSensorIsPressed());


        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.addData("ARGB", colorSensor.argb());
        telemetry.addData("Distance", colorSensor.getDistance(DistanceUnit.CM));



        telemetry.update();


    }

    @Override
    public void stop() {

        super.stop();
    }

    public void drivetrain() {

        double horizontal = 0; // x-axis movement
        double vertical = 0; // y-axis movement
        double spin = 0; // rotational movement

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

    public void intake() {

        colorSensor2.enableLed(false);

        // Outtake
        if (gamepad2.right_trigger > 0) {
            intake.setPower(intakeSpeed);
        }
        // Intake
        else if (gamepad2.left_trigger > 0) {
            intake.setPower(-intakeSpeed);

           /*
            if (colorSensor.getDistance(DistanceUnit.CM) <= 3.5) {

                intake.setPower(0);
                colorSensor2.enableLed(true);
            }
            */
        }
        else {
            // Outtake
            if (gamepad1.right_trigger > 0) {
                intake.setPower(intakeSpeed);
            }
            // Intake
            else if (gamepad1.left_trigger > 0){
                intake.setPower(-intakeSpeed);

                /*
                if (colorSensor.getDistance(DistanceUnit.CM) <= 3.5) {

                    intake.setPower(0);
                    colorSensor2.enableLed(true);
                }
                */
            }
            else {
                intake.setPower(0);
            }
        }
    }

    public void setTurret() {

        if (gamepad2.x) {
            turret.setPower(-turretSpeed);

            if (leftSwitchIsPressed()) {
                turret.setPower(0);
            }
        }
        else if (gamepad2.b) {
            turret.setPower(turretSpeed);

            if (rightSwitchIsPressed()) {
                turret.setPower(0);
            }
        }
        else {
            if (gamepad1.dpad_left) {
                turret.setPower(-turretSpeed);

                if (leftSwitchIsPressed()) {
                    turret.setPower(0);
                }
            }
            else if (gamepad1.dpad_right) {
                turret.setPower(turretSpeed);

                if (rightSwitchIsPressed()) {
                    turret.setPower(0);
                }
            }
            else {
                turret.setPower(0);
            }
        }
    }

    public void setArm() {

        // Raises arm
        if (gamepad2.dpad_up) {
            armMotor.setPower(armMotorSpeed);

            if (touchSensorIsPressed()) {
                armMotor.setPower(0);
            }
        }
        // Lowers arm
        else if (gamepad2.dpad_down) {
            armMotor.setPower(-armMotorSpeed);
        }
        else {

            if (gamepad1.dpad_up) {
                armMotor.setPower(armMotorSpeed);

                if (touchSensorIsPressed()) {
                    armMotor.setPower(0);
                }
            }
            else if (gamepad1.dpad_down) {
                armMotor.setPower(-armMotorSpeed);
            }
            else {
                armMotor.setPower(0);
            }
        }
    }

    public void carouselWheel() {

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

    public void setCapper() {

        if (gamepad1.y) {
            capper.setPower(-0.5);
        }
        else if (gamepad1.a) {
            capper.setPower(0.5);
        }
        else {
            capper.setPower(0);
        }
    }

    // Test code didnt work
    public void rumble() {
        if (runTime.seconds() == 75) {
            gamepad1.rumble(2000);
        }
    }

    // Test code didnt work
    public void colorSensor() {

        colorSensor2.enableLed(false);

      if (colorSensor.getDistance(DistanceUnit.CM) <= 3.5) {

          intake.setPower(0);
          colorSensor2.enableLed(true);
      }
    }

    // Test code didnt work
    public void carouselWheelRampUpTest() {

        if (gamepad1.x) {

            if (rampUp) {
                carouselWheelLeft.setPower(carouselWheelSpeed += INCREMENT);
            }
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

    public boolean leftSwitchIsPressed() {
        return leftTurretLimit.isPressed();
    }
    public boolean rightSwitchIsPressed() {
        return rightTurretLimit.isPressed();
    }
    public boolean touchSensorIsPressed() { return maxArmHeightLimit.isPressed(); }
}
