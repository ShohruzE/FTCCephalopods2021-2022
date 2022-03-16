package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "8087 park- Blue",group = "8087")
public class parkauto_8087_Blue extends LinearOpMode {

    // Drivetrain Motors
    DcMotorEx frontLeft = null;
    DcMotorEx frontRight = null;
    DcMotorEx backLeft = null;
    DcMotorEx backRight = null;

    DcMotor intakeLeftMotor = null;
    DcMotor intakeRightMotor = null;
    DcMotor armMotor = null;
    DcMotor carouselWheel = null;

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double SIDE_SPEED = 0.6;
    static final double INTAKE_SPEED = 1.0;
    static final double ARM_SPEED = 0.6;


    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;

    private Orientation lastAngles = new Orientation();
    private double currentAngle = 0.0;


    static final int TICKS_PER_REVOLUTION = 1120; // REV HD HEX 40:1
    static final double DRIVE_GEAR_REDUCTION = 1; //TODO: CALCULATE GEAR RATIO
    static final double WHEEL_DIAMETER_INCHES = 3.93701;
    static double COUNTS_PER_INCH = (TICKS_PER_REVOLUTION * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

        // Motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        frontRight = hardwareMap.get(DcMotorEx.class, "FR");
        backLeft = hardwareMap.get(DcMotorEx.class, "BL");
        backRight = hardwareMap.get(DcMotorEx.class, "BR");

        intakeLeftMotor = hardwareMap.get(DcMotor.class, "IL");
        intakeRightMotor = hardwareMap.get(DcMotor.class, "IR");
        armMotor = hardwareMap.get(DcMotor.class, "ARM");
        carouselWheel = hardwareMap.get(DcMotor.class, "CW");

        // Set motor direction
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d, %7d, %7d, %7d",
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                backLeft.getCurrentPosition(),
                backRight.getCurrentPosition());
        telemetry.update();


        waitForStart();// TODO: PLACE AUTONOMOUS CODE AFTER THIS LINE

        runtime.reset();


        //go forward
        frontLeft.setPower(FORWARD_SPEED);
        backLeft.setPower(FORWARD_SPEED);
        frontRight.setPower(FORWARD_SPEED);
        backRight.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();


        }

        //turn left
        frontLeft.setPower(TURN_SPEED);
        backLeft.setPower(TURN_SPEED);
        frontRight.setPower(-TURN_SPEED);
        backRight.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //go forward
        frontLeft.setPower(FORWARD_SPEED);
        backLeft.setPower(FORWARD_SPEED);
        frontRight.setPower(FORWARD_SPEED);
        backRight.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.8)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();


        }


        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);

    }
}
