package org.firstinspires.ftc.teamcode.FTC_9385;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PIDController;

@Disabled
@Autonomous(name = "Autonomous_BlueLeft - Team_9385", group = "Autonomous - 9385")
public class Autonomous_9385_BlueLeft extends LinearOpMode {

    DcMotorEx frontLeft = null;
    DcMotorEx frontRight = null;
    DcMotorEx backLeft = null;
    DcMotorEx backRight = null;

    DcMotor intakeMotor = null;
    DcMotor arm = null;
    DcMotor carouselWheelLeft = null;
    DcMotor carouselWheelRight = null;

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
        frontLeft          = hardwareMap.get(DcMotorEx.class, "FL");
        frontRight         = hardwareMap.get(DcMotorEx.class, "FR");
        backLeft           = hardwareMap.get(DcMotorEx.class, "BL");
        backRight          = hardwareMap.get(DcMotorEx.class, "BR");

        intakeMotor        = hardwareMap.get(DcMotor.class, "Intake");
        arm                = hardwareMap.get(DcMotor.class, "Arm");
        carouselWheelLeft  = hardwareMap.get(DcMotor.class, "CWL");
        carouselWheelRight = hardwareMap.get(DcMotor.class, "CWR");


        // Set motor direction
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        carouselWheelLeft.setDirection(DcMotor.Direction.FORWARD);
        carouselWheelRight.setDirection(DcMotor.Direction.FORWARD);


        // Set ZERO POWER BEHAVIOR
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselWheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselWheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

        runtime.reset();




        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 17.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        frontLeft.setPower(0.9);
        frontRight.setPower(0.9);
        backLeft.setPower(0.9);
        backRight.setPower(0.9);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        frontLeft.setPower(5);
        frontRight.setPower(-5);
        backLeft.setPower(5);
        backRight.setPower(-5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        frontLeft.setPower(0.6);
        frontRight.setPower(0.6);
        backLeft.setPower(0.6);
        backRight.setPower(0.6);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.45)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        frontLeft.setPower(-5);
        frontRight.setPower(5);
        backLeft.setPower(-5);
        backRight.setPower(5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.75)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        frontLeft.setPower(0.6);
        frontRight.setPower(0.6);
        backLeft.setPower(0.6);
        backRight.setPower(0.6);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.45)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

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
            frontRight.setTargetPosition(-frontRightTarget);
            backLeft.setTargetPosition(-backLeftTarget);
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

    public void strafeDrive(double speed, double leftInches, double rightInches) {

        int frontLeftTarget = 0;
        int frontRightTarget = 0;
        int backLeftTarget = 0;
        int backRightTarget = 0;

        // Calculates the distance that the robot has to move
        frontLeftTarget = frontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        frontRightTarget = frontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        backLeftTarget = backLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        backRightTarget = backRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

        // Sets the position for the robot to move to
        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(-frontRightTarget);
        backLeft.setTargetPosition(-backLeftTarget);
        backRight.setTargetPosition(backRightTarget);

        // Runs robot to that position and stops once complete.
        runToPosition();

        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        runUsingEncoders();

        resetEncoders();
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

    public void spinLeftCarousel(double speed, int time) {

        carouselWheelLeft.setPower(speed);
        sleep(time);
    }

    public void spinRightCarousel(double speed, int time) {

        carouselWheelRight.setPower(speed);
        sleep(time);
    }

    public void intake(double speed, int time) {

        intakeMotor.setPower(speed);
        sleep(time);
    }

    public void moveArm(double speed, int time) {

        arm.setPower(speed);
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
