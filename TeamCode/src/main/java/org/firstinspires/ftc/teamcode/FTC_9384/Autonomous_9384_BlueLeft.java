package org.firstinspires.ftc.teamcode.FTC_9384;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PIDController;

@Disabled
@Autonomous(name = "Autonomous_BlueLeft - Team_9384", group = "Autonomous - 9384")
public class Autonomous_9384_BlueLeft extends LinearOpMode {

    // Motors
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

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
    BNO055IMU imu;

    private Orientation lastAngles = new Orientation();
    private double currentAngle = 0.0;

    static final int TICKS_PER_REVOLUTION = 1120; // REV HD HEX 40:1
    static final double DRIVE_GEAR_REDUCTION = 1; //TODO: CALCULATE GEAR RATIO
    static final double WHEEL_DIAMETER_INCHES = 2.95275;
    static double COUNTS_PER_INCH = (TICKS_PER_REVOLUTION * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft  = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft   = hardwareMap.get(DcMotor.class, "BL");
        backRight  = hardwareMap.get(DcMotor.class, "BR");


        intake          = hardwareMap.get(CRServo.class, "Intake");
        elevator        = hardwareMap.get(DcMotor.class, "EL");
        bucket          = hardwareMap.get(Servo.class, "B");
        carouselWheel   = hardwareMap.get(DcMotor.class, "CW");


        // Set motor direction
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        intake.setDirection(CRServo.Direction.REVERSE);
        elevator.setDirection(DcMotor.Direction.REVERSE);
        carouselWheel.setDirection(DcMotor.Direction.FORWARD);


        // Set ZERO POWER BEHAVIOR
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bucket.setPosition(BUCKET_START);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();


        // Set up encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d, %7d, %7d, %7d",
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                backLeft.getCurrentPosition(),
                backRight.getCurrentPosition());
        telemetry.update();

        waitForStart();// TODO: PLACE AUTONOMOUS CODE AFTER THIS LINE

        runTime.reset();

        encoderDrive(20,20,20,20,0.5);

    }

    public void encoderDrive(double speed, double frontLeftInches, double frontRightInches,
                             double backLeftInches, double backRightInches) {

        int frontLeftTarget = 0;
        int frontRightTarget = 0;
        int backLeftTarget = 0;
        int backRightTarget = 0;

        if (opModeIsActive()) {

            // Calculates the distance that the robot has to move
            frontLeftTarget = frontLeft.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
            frontRightTarget = frontRight.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
            backLeftTarget = backLeft.getCurrentPosition() + (int) (backLeftInches * COUNTS_PER_INCH);
            backRightTarget = backRight.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);

            // Sets the position for the robot to move to
            frontLeft.setTargetPosition(frontLeftTarget);
            frontRight.setTargetPosition(frontRightTarget);
            backLeft.setTargetPosition(backLeftTarget);
            backRight.setTargetPosition(backRightTarget);

            // Runs robot to that position and stops once complete.
            runToPosition();

            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d %7d :%7d", frontLeftTarget,
                        frontRightTarget, backLeftTarget, backRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d %7d :%7d",
                        frontLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition(),
                        backLeft.getCurrentPosition(),
                        backRight.getCurrentPosition());
                telemetry.update();
            }

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            runUsingEncoders();
        }
    }

    public void resetEncoders() {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoders() {

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runToPosition() {

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // ROBOT AUTONOMOUS FUNCTIONS

    public void spinCarousel(double speed, int time) {

        carouselWheel.setPower(speed);
        sleep(time);
    }

    public void intake(double speed, int time) {

        intake.setPower(speed);
        sleep(time);
    }

    public void moveArm(double speed, int time) {


        sleep(time);
    }


    // ROBOT IMU FUNCTIONS

    public void resetAngle() {

        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentAngle = 0;
    }

    public double getAngle() {

        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        if (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        else if (deltaAngle <= 360) {
            deltaAngle += 360;
        }

        currentAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currentAngle;
    }

    public void turn(double degrees) {

        resetAngle();

        double error = degrees;


        while (opModeIsActive() && Math.abs(error) > 2) {

            double motorPower;

            if (error < 0) {
                motorPower = -0.3;
            }
            else {
                motorPower = 0.3;
            }
            frontLeft.setPower(-motorPower);
            frontRight.setPower(motorPower);
            backLeft.setPower(-motorPower);
            backRight.setPower(motorPower);

            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

        }
    }

    public void turnTo(double degrees) {

        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double error = degrees - orientation.firstAngle;

        if (error > 180) {
            error -= 360;
        }
        else if (error < -180) {
            error += 360;
        }

        turn(error);
    }

    public double getAbsoluteAngle() {

        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    void turnToPID(double targetAngle) {

        PIDController pid = new PIDController(targetAngle,0.01, 0, 0.003);
        while(opModeIsActive() && Math.abs(targetAngle - getAbsoluteAngle()) > 1) {

            double motorPower = pid.update(getAbsoluteAngle());

            frontLeft.setPower(-motorPower);
            frontRight.setPower(motorPower);
            backLeft.setPower(-motorPower);
            backRight.setPower(motorPower);
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    void turnPID(double degrees) {

        turnToPID(degrees + getAbsoluteAngle());
    }
}
