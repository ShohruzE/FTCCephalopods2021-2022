package org.firstinspires.ftc.teamcode.FTC_8087;

import android.hardware.Sensor;
import android.widget.Switch;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Shohruz on 10/10/21.
 */

@TeleOp(name = "FSM Test - Team_8087", group = "TeleOp")
// @Disabled
public class FiniteStateMachineTest extends OpMode {

    public enum ArmState {
        ARM_START,
        ARM_MOVING,
        ARM_RAISED,
        ARM_RESET
    }
    public enum TurretState {
        TURRET_START,
        TURRET_MOVING,
        TURRET_REACHED,
        TURRET_RESET
    }

    boolean armLow = false;
    boolean armMid = false;
    boolean armHigh = false;

    boolean turretRight = false;
    boolean turretLeft = false;

    ArmState armState = ArmState.ARM_START;
    TurretState turretState = TurretState.TURRET_START;

    public final double ARM_LOW_TIME = 0.8;
    public final double ARM_MID_TIME = 1.3;
    public final double ARM_HIGH_TIME = 2.0;
    public final double TURRET_ROTATE_TIME = 1.2;

    public final int ARM_LOW = 725;
    public final int ARM_MID = 1500;
    public final int ARM_HIGH = 2500;
    public final int TURRET_RIGHT = 150;
    public final int TURRET_LEFT = -150;


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

    //   ColorSensor colorSensor;
    RevTouchSensor leftTurretLimit;
    RevTouchSensor rightTurretLimit;
    RevTouchSensor maxArmHeightLimit;

    // Motor Speed
    double frontLeftMotorSpeed;
    double frontRightMotorSpeed;
    double backLeftMotorSpeed;
    double backRightMotorSpeed;

    double intakeSpeed = 1;
    double turretSpeed = 0.5;
    double armMotorSpeed = 0.6;
    double carouselWheelSpeed = 1.0;

    double powerMultiplier = 1.0;

    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime armTimer = new ElapsedTime();
    private ElapsedTime turretTimer = new ElapsedTime();
    private ElapsedTime outtakeTimer = new ElapsedTime();


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

        //      colorSensor      = hardwareMap.get(ColorSensor.class, "color");
        leftTurretLimit = hardwareMap.get(RevTouchSensor.class, "LTL");
        rightTurretLimit = hardwareMap.get(RevTouchSensor.class, "RTL");
        maxArmHeightLimit = hardwareMap.get(RevTouchSensor.class, "MAHL");


        // Set motor direction
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
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

        runTime.reset();
        armTimer.reset();
        turretTimer.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.update();
    }

    @Override
    public void loop() {

        switch (armState) {

            case ARM_START:
                if (gamepad2.dpad_down) {

                    armLow = true;
                    turretLeft = true;
                    armMotor.setTargetPosition(ARM_LOW);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(armMotorSpeed);
                    armState = ArmState.ARM_RAISED;
                }
                else if (gamepad2.dpad_left) {

                    armMid = true;
                    turretLeft = true;
                    armMotor.setTargetPosition(ARM_MID);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(armMotorSpeed);
                    armState = ArmState.ARM_RAISED;
                }
                else if (gamepad2.dpad_up) {

                    armHigh = true;
                    turretLeft = true;
                    armMotor.setTargetPosition(ARM_HIGH);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(armMotorSpeed);
                    armState = ArmState.ARM_RAISED;
                }
                else if (gamepad2.a) {

                    armLow = true;
                    turretRight = true;
                    armMotor.setTargetPosition(ARM_LOW);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(armMotorSpeed);
                    armState = ArmState.ARM_RAISED;
                }
                else if (gamepad2.b) {

                    armMid = true;
                    turretRight = true;
                    armMotor.setTargetPosition(ARM_MID);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(armMotorSpeed);
                    armState = ArmState.ARM_RAISED;
                }
                else if (gamepad2.y) {

                    armHigh = true;
                    turretRight = true;
                    armMotor.setTargetPosition(ARM_HIGH);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(armMotorSpeed);
                    armState = ArmState.ARM_RAISED;
                }
                break;

            case ARM_RAISED:

                if (Math.abs(armMotor.getCurrentPosition() - ARM_LOW) < 25 && armLow) {
                    armMotor.setPower(0);
                    armTimer.reset();
                    turretState = TurretState.TURRET_START;

                }
                else if (Math.abs(armMotor.getCurrentPosition() - ARM_MID) < 25 && armMid) {
                    armMotor.setPower(0);
                    armTimer.reset();
                    turretState = TurretState.TURRET_START;
                }
                else if (Math.abs(armMotor.getCurrentPosition() - ARM_HIGH) < 25 && armHigh) {
                    armMotor.setPower(0);
                    armTimer.reset();
                    turretState = TurretState.TURRET_START;
                }
                break;

            case ARM_RESET:

                if (Math.abs(armMotor.getCurrentPosition()) < 25) {

                    armState = ArmState.ARM_START;
                }
                break;

            default:
                armState = ArmState.ARM_START;

        }

        switch (turretState) {

            case TURRET_START:

                if (armLow && turretRight) {
                    turret.setTargetPosition(TURRET_RIGHT);
                    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    turret.setPower(turretSpeed);
                    outtakeTimer.reset();
                    turretState = TurretState.TURRET_REACHED;
                }
                else if (armMid && turretRight) {
                    turret.setTargetPosition(TURRET_RIGHT);
                    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    turret.setPower(turretSpeed);
                    outtakeTimer.reset();
                    turretState = TurretState.TURRET_REACHED;
                }
                else if (armHigh && turretRight) {
                    turret.setTargetPosition(TURRET_RIGHT);
                    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    turret.setPower(turretSpeed);
                    outtakeTimer.reset();
                    turretState = TurretState.TURRET_REACHED;
                }
                else if (armLow && turretLeft) {
                    turret.setTargetPosition(TURRET_LEFT);
                    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    turret.setPower(turretSpeed);
                    outtakeTimer.reset();
                    turretState = TurretState.TURRET_REACHED;
                }
                else if (armMid && turretLeft) {
                    turret.setTargetPosition(TURRET_LEFT);
                    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    turret.setPower(turretSpeed);
                    outtakeTimer.reset();
                    turretState = TurretState.TURRET_REACHED;
                }
                else if (armHigh && turretLeft) {
                    turret.setTargetPosition(TURRET_LEFT);
                    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    turret.setPower(turretSpeed);
                    outtakeTimer.reset();
                    turretState = TurretState.TURRET_REACHED;
                }
                break;

            case TURRET_REACHED:

                if (outtakeTimer.seconds() >= 2) {

                    turret.setTargetPosition(0);
                    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    turret.setPower(turretSpeed);
                    turretState = TurretState.TURRET_RESET;
                }
                break;

            case TURRET_RESET:

                if (Math.abs(turret.getCurrentPosition()) < 5) {
                    armMotor.setTargetPosition(0);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(armMotorSpeed);
                    armState = ArmState.ARM_RESET;
                }
                break;

            default:
                turretState = TurretState.TURRET_START;

        }

        // TODO: FIX TURRET STATE
        if (gamepad2.x && (armState != ArmState.ARM_START) && (turretState != TurretState.TURRET_START)) {

            armLow = false;
            armMid = false;
            armHigh = false;

            turretRight = false;
            turretLeft = false;

            if (turretTimer.seconds() >= TURRET_ROTATE_TIME) {
                armState = ArmState.ARM_START;
            }
        }

        drivetrain();
        intake();
        carouselWheel();

        setCapper();

        //     colorSensor();


        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.addData("Motors", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), " +
                "backRight (%.2f)", frontLeftMotorSpeed, frontRightMotorSpeed, backLeftMotorSpeed, backRightMotorSpeed);
        telemetry.addData("Turret: ", turret.getCurrentPosition());
        telemetry.addData("Arm: ", armMotor.getCurrentPosition());

        telemetry.addData("Left Switch,", leftSwitchIsPressed());
        telemetry.addData("Right Switch,", rightSwitchIsPressed());
        telemetry.addData("Max arm Height,", touchSensorIsPressed());

        /*
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.addData("Hue", colorSensor.argb());
        telemetry.update();

         */
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

        // Fast outtake
        if (gamepad2.right_trigger > 0) {
            intake.setPower(intakeSpeed);
        }
        // Fast intake
        else if (gamepad2.left_trigger > 0){
            intake.setPower(-intakeSpeed);
        }
        // Slow outtake
        else if (gamepad2.right_bumper) {
            intake.setPower(0.5);
        }
        // Slow intake
        else if (gamepad2.left_bumper) {
            intake.setPower(-0.5);
        }
        else {
            intake.setPower(0);
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
            capper.setPower(0.5);
        }
        else if (gamepad1.a) {
            capper.setPower(-0.5);
        }
        else {
            capper.setPower(0);
        }
    }

    //  public void colorSensor() {}

    public boolean leftSwitchIsPressed() {
        return leftTurretLimit.isPressed();
    }
    public boolean rightSwitchIsPressed() {
        return rightTurretLimit.isPressed();
    }
    public boolean touchSensorIsPressed() { return maxArmHeightLimit.isPressed(); }
}
