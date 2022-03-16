package org.firstinspires.ftc.teamcode.FTC_8088;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

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
@Autonomous(name = "Autonomous - Team_8088", group = "Autonomous - 8088")
public class Autonomous_8088 extends LinearOpMode {

    DcMotorEx frontLeft = null;
    DcMotorEx frontRight = null;
    DcMotorEx backLeft = null;
    DcMotorEx backRight = null;

    DcMotor intake = null;
    DcMotor turretPosition = null;
    DcMotor turretRotation = null;
    DcMotor carouselWheel = null;

    // Field location variables
    boolean blueLeft = false;
    boolean blueRight = false;
    boolean redLeft = false;
    boolean redRight = false;


    ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;

    private Orientation lastAngles = new Orientation();
    private double currentAngle = 0.0;


    OpenCvCamera webcam;
    ObjectDetectionEOCV detector;


    static final int TICKS_PER_REVOLUTION = 1120; // REV HD HEX 40:1
    static final double DRIVE_GEAR_REDUCTION = 125/72; //TODO: CALCULATE GEAR RATIO
    static final double WHEEL_DIAMETER_INCHES = 3.93701;
    static double COUNTS_PER_INCH = (TICKS_PER_REVOLUTION * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

        // IMU initialization
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        // Motors
        frontLeft       = hardwareMap.get(DcMotorEx.class, "FL");
        frontRight      = hardwareMap.get(DcMotorEx.class, "FR");
        backLeft        = hardwareMap.get(DcMotorEx.class, "BL");
        backRight       = hardwareMap.get(DcMotorEx.class, "BR");

        intake          = hardwareMap.get(DcMotor.class, "Intake");
        turretPosition  = hardwareMap.get(DcMotor.class, "TP");
        turretRotation  = hardwareMap.get(DcMotor.class, "TR");
        carouselWheel   = hardwareMap.get(DcMotor.class, "CW");


        // Set motor direction
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.FORWARD);
        turretRotation.setDirection(DcMotor.Direction.FORWARD);
        turretPosition.setDirection(DcMotor.Direction.FORWARD);
        carouselWheel.setDirection(DcMotor.Direction.FORWARD);

        // Set ZERO POWER BEHAVIOR
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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


        telemetry.addData("Where are you on the field?", "Press X for blueLeft " +
                                                                            "A for blueRight " +
                                                                            "B for redLeft " +
                                                                            "Y for redRight");
        telemetry.update();

        if (gamepad1.x) {
            blueLeft = true;
            telemetry.addData("You are at", "blueLeft");
            telemetry.update();
        }
        else if (gamepad1.a) {
            blueRight = true;
            telemetry.addData("You are at", "blueRight");
            telemetry.update();
        }
        else if (gamepad1.b) {
            redLeft = true;
            telemetry.addData("You are at", "redLeft");
            telemetry.update();
        }
        else if (gamepad1.y) {
            redRight = true;
            telemetry.addData("You are at", "redRight");
            telemetry.update();
        }


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(detector);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {

                telemetry.addData("Error: ", errorCode);
                telemetry.update();
            }
        });

        telemetry.addData("Duck Position: ", detector.getDuckPosition());
        telemetry.update();


        waitForStart();// TODO: PLACE AUTONOMOUS CODE AFTER THIS LINE

        runtime.reset();


        while (opModeIsActive()) {


            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();


            // RUN CODE IF ROBOT IS ON LEFT SIDE OF BLUE ALLIANCE
            if (blueLeft) {

                // Run code if duck is on the left
                if (detector.getDuckPosition() == 0) {

                    encoderDrive(-0.4, 20, 20);
                    moveArm(0.2, 500);
                    turn(45);
                    encoderDrive(0.2, 8, 8);
                    conveyorBelt(0.5, 2000);
                    encoderDrive(0.4, 8, 8);
                    turn(45);
                    encoderDrive(1, 35, 35);

                }
                // Run code if duck is in the middle
                else if (detector.getDuckPosition() == 1) {

                    encoderDrive(-0.4, 20, 20);
                    moveArm(0.2, 1000);
                    turn(45);
                    encoderDrive(0.2, 8, 8);
                    conveyorBelt(0.5, 2000);
                    encoderDrive(0.4, 8, 8);
                    turn(45);
                    encoderDrive(1, 35, 35);

                }
                // Run code if duck is on the right
                else if (detector.getDuckPosition() == 2) {

                    encoderDrive(-0.4, 20, 20);
                    moveArm(0.2, 1500);
                    turn(45);
                    encoderDrive(0.2, 8, 8);
                    conveyorBelt(0.5, 2000);
                    encoderDrive(0.4, 8, 8);
                    turn(45);
                    encoderDrive(1, 35, 35);

                }
                else {

                    encoderDrive(-0.4, 20, 20);
                    moveArm(0.2, 1500);
                    turn(45);
                    encoderDrive(0.2, 8, 8);
                    conveyorBelt(0.5, 2000);
                    encoderDrive(0.4, 8, 8);
                    turn(45);
                    encoderDrive(1, 35, 35);
                }
            }

            // RUN CODE IF ROBOT IS ON RIGHT SIDE OF BLUE ALLIANCE
            else if (blueRight) {

                // Run code if duck is on the left
                if (detector.getDuckPosition() == 0) {

                    encoderDrive(-0.4, 18,18);
                    moveArm(0.2, 500);
                    turn(45);
                    encoderDrive(-0.2, 10, 10);
                    conveyorBelt(0.5, 2000);
                    encoderDrive(0.2, 10, 10);
                    turn(45);
                    encoderDrive(0.4, 25, 25);
                    strafeDrive(0.2, 7,7);
                    spinCarousel(0.8, 3000);
                    strafeDrive(0.4,15, 15);
                    encoderDrive(0.4, 5, 5);
                    turn(-180);
                    encoderDrive(0.2, 6, 6);

                }
                // Run code if duck is in the middle
                else if (detector.getDuckPosition() == 1) {

                    encoderDrive(-0.4, 18,18);
                    moveArm(0.2, 1000);
                    turn(45);
                    encoderDrive(-0.2, 10, 10);
                    conveyorBelt(0.5, 2000);
                    encoderDrive(0.2, 10, 10);
                    turn(45);
                    encoderDrive(0.4, 25, 25);
                    strafeDrive(0.2, 7,7);
                    spinCarousel(0.8, 3000);
                    strafeDrive(0.4,15, 15);
                    encoderDrive(0.4, 5, 5);
                    turn(-180);
                    encoderDrive(0.2, 6, 6);

                }
                // Run code if duck is on the right
                else if (detector.getDuckPosition() == 2) {

                    encoderDrive(-0.4, 18,18);
                    moveArm(0.2, 1500);
                    turn(45);
                    encoderDrive(-0.2, 10, 10);
                    conveyorBelt(0.5, 2000);
                    encoderDrive(0.2, 10, 10);
                    turn(45);
                    encoderDrive(0.4, 25, 25);
                    strafeDrive(0.2, 7,7);
                    spinCarousel(0.8, 3000);
                    strafeDrive(0.4,15, 15);
                    encoderDrive(0.4, 5, 5);
                    turn(-180);
                    encoderDrive(0.2, 6, 6);

                }
                else {

                    encoderDrive(-0.4, 18,18);
                    moveArm(0.2, 1500);
                    turn(45);
                    encoderDrive(-0.2, 10, 10);
                    conveyorBelt(0.5, 2000);
                    encoderDrive(0.2, 10, 10);
                    turn(45);
                    encoderDrive(0.4, 25, 25);
                    strafeDrive(0.2, 7,7);
                    spinCarousel(0.8, 3000);
                    strafeDrive(0.4,15, 15);
                    encoderDrive(0.4, 5, 5);
                    turn(-180);
                    encoderDrive(0.2, 6, 6);
                }
            }

            // RUN CODE IF ROBOT IS ON LEFT SIDE OF RED ALLIANCE
            else if (redLeft) {

                // Run code if duck is on the left
                if (detector.getDuckPosition() == 0) {

                    encoderDrive(-0.4, 20, 20);
                    moveArm(0.2, 500);
                    turn(55);
                    encoderDrive(-0.2, 10, 10);
                    conveyorBelt(0.5, 2000);
                    encoderDrive(0.4, 10, 10);
                    turn(-55);
                    strafeDrive(0.4, 27, 27);
                    encoderDrive(0.2, 7, 7);
                    spinCarousel(0.8, 3000);
                    encoderDrive(0.4, 15, 15);
                    strafeDrive(0.4, -8,-8);
                    turn(-90);
                    encoderDrive(0.3, 8, 8);

                }
                // Run code if duck is in the middle
                else if (detector.getDuckPosition() == 1) {

                    encoderDrive(-0.4, 20, 20);
                    moveArm(0.2, 1000);
                    turn(55);
                    encoderDrive(-0.2, 10, 10);
                    conveyorBelt(0.5, 2000);
                    encoderDrive(0.4, 10, 10);
                    turn(-55);
                    strafeDrive(0.4, 27, 27);
                    encoderDrive(0.2, 7, 7);
                    spinCarousel(0.8, 3000);
                    encoderDrive(0.4, 15, 15);
                    strafeDrive(0.4, -8,-8);
                    turn(-90);
                    encoderDrive(0.3, 8, 8);

                }
                // Run code if duck is on the right
                else if (detector.getDuckPosition() == 2){

                    encoderDrive(-0.4, 20, 20);
                    moveArm(0.2, 1500);
                    turn(55);
                    encoderDrive(-0.2, 10, 10);
                    conveyorBelt(0.5, 2000);
                    encoderDrive(0.4, 10, 10);
                    turn(-55);
                    strafeDrive(0.4, 27, 27);
                    encoderDrive(0.2, 7, 7);
                    spinCarousel(0.8, 3000);
                    encoderDrive(0.4, 15, 15);
                    strafeDrive(0.4, -8,-8);
                    turn(-90);
                    encoderDrive(0.3, 8, 8);

                }
                else {

                    encoderDrive(-0.4, 20, 20);
                    moveArm(0.2, 1500);
                    turn(55);
                    encoderDrive(-0.2, 10, 10);
                    conveyorBelt(0.5, 2000);
                    encoderDrive(0.4, 10, 10);
                    turn(-55);
                    strafeDrive(0.4, 27, 27);
                    encoderDrive(0.2, 7, 7);
                    spinCarousel(0.8, 3000);
                    encoderDrive(0.4, 15, 15);
                    strafeDrive(0.4, -8,-8);
                    turn(-90);
                    encoderDrive(0.3, 8, 8);
                }
            }

            // RUN CODE IF ROBOT IS ON RIGHT SIDE OF RED ALLIANCE
            else if (redRight) {

                // Run code if duck is on the left
                if (detector.getDuckPosition() == 0) {

                    encoderDrive(-0.4, 20, 20);
                    moveArm(0.2, 500);
                    turn(-45);
                    encoderDrive(-0.2, 8, 8);
                    conveyorBelt(0.5, 2000);
                    encoderDrive(0.4, 8, 8);
                    turn(45);
                    encoderDrive(1, 35, 35);

                }
                // Run code if duck is in the middle
                else if (detector.getDuckPosition() == 1) {

                    encoderDrive(-0.4, 20, 20);
                    moveArm(0.2, 1000);
                    turn(-45);
                    encoderDrive(-0.2, 8, 8);
                    conveyorBelt(0.5, 2000);
                    encoderDrive(0.4, 8, 8);
                    turn(45);
                    encoderDrive(1, 35, 35);
                }
                // Run code if duck is on the right
                else if (detector.getDuckPosition() == 2){

                    encoderDrive(-0.4, 20, 20);
                    moveArm(0.2, 1500);
                    turn(-45);
                    encoderDrive(-0.2, 8, 8);
                    conveyorBelt(0.5, 2000);
                    encoderDrive(0.4, 8, 8);
                    turn(45);
                    encoderDrive(1, 35, 35);
                }
                else {

                    encoderDrive(-0.4, 20, 20);
                    moveArm(0.2, 1500);
                    turn(-45);
                    encoderDrive(-0.2, 8, 8);
                    conveyorBelt(0.5, 2000);
                    encoderDrive(0.4, 8, 8);
                    turn(45);
                    encoderDrive(1, 35, 35);
                }
            }

            sleep(100);
        }

    }

    public void encoderDrive(double speed, double leftInches, double rightInches) {

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
        frontRight.setTargetPosition(frontRightTarget);
        backLeft.setTargetPosition(backLeftTarget);
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

        turretPosition.setPower(speed);
        sleep(time);
    }

    public void conveyorBelt(double speed, int time) {

        turretRotation.setPower(speed);
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
