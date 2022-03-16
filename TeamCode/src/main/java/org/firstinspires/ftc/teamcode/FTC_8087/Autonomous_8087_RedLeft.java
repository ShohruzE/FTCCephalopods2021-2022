package org.firstinspires.ftc.teamcode.FTC_8087;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ObjectDetectionEOCV;
import org.firstinspires.ftc.teamcode.PIDController;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "AutonomousRedLeft - 8087", group = "Autonomous - 8087")
public class Autonomous_8087_RedLeft extends LinearOpMode {

    // Drivetrain Motors
    DcMotor frontLeft = null;
    DcMotor frontRight = null;
    DcMotor backLeft = null;
    DcMotor backRight = null;

    CRServo intake;
    DcMotor armMotor;
    DcMotor turret;
    DcMotor carouselWheel;


    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;

    private Orientation lastAngles = new Orientation();
    private double currentAngle = 0.0;


    static final double TICKS_PER_REVOLUTION = 537.6; // Neverest 20:1
    static final double DRIVE_GEAR_REDUCTION = 1; //TODO: CALCULATE GEAR RATIO
    static final double WHEEL_DIAMETER_INCHES = 2.95275;
    static double COUNTS_PER_INCH = (TICKS_PER_REVOLUTION * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft   = hardwareMap.get(DcMotor.class, "FL");
        frontRight  = hardwareMap.get(DcMotor.class, "FR");
        backLeft    = hardwareMap.get(DcMotor.class, "BL");
        backRight   = hardwareMap.get(DcMotor.class, "BR");

        intake           = hardwareMap.get(CRServo.class, "Intake");
        turret           = hardwareMap.get(DcMotor.class, "TR");
        armMotor         = hardwareMap.get(DcMotor.class, "ARM");
        carouselWheel    = hardwareMap.get(DcMotor.class, "CW");


        // Set motor direction
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set ZERO POWER BEHAVIOR
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();// TODO: PLACE AUTONOMOUS CODE AFTER THIS LINE && COPY AND PASTE WEBCAM CODE TO OTHER AUTONOMOUS FILES

        runtime.reset();



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

    public void moveArm(double speed, int time) {

        armMotor.setPower(speed);
        sleep(time);
    }

    public void intake(double speed, int time) {

        intake.setPower(speed);
        sleep(time);

    }

    public void setTurret(double speed, int time) {

        turret.setPower(speed);
        sleep(time);
    }

    // IMU functions

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
