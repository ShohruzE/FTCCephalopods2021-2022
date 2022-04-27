package org.firstinspires.ftc.teamcode.FTC_8088;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Barcode;
import org.firstinspires.ftc.teamcode.Scanner;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunnerCancelable;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class AutoAsyncTest extends LinearOpMode {

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
    CRServo guideWheelLeft;
    CRServo guideWheelRight;

    CRServo capper;
    CRServo capper2;

    RevTouchSensor leftTurretLimit;
    RevTouchSensor rightTurretLimit;
    RevTouchSensor lowArmHeightLimit;
    RevColorSensorV3 colorSensor;

    DigitalChannel LED_LRed;
    DigitalChannel LED_LGreen;
    DigitalChannel LED_RRed;
    DigitalChannel LED_RGreen;


    // Motor Speed
    double frontLeftMotorSpeed;
    double frontRightMotorSpeed;
    double backLeftMotorSpeed;
    double backRightMotorSpeed;

    double intakeSpeed = 1;
    double turretSpeed = 0.25;
    double armMotorSpeed = 0.4;
    double carouselWheelSpeed = 0.6;

    double powerMultiplier = 1.0;

    //boolean hasCargo = colorSensor.getDistance(DistanceUnit.CM) <= 4.5;
    boolean intakeState = true;


    ElapsedTime runtime = new ElapsedTime();


    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {


        turret           = hardwareMap.get(DcMotor.class, "TR");
        armMotor         = hardwareMap.get(DcMotor.class, "ARM");
        carouselWheelLeft    = hardwareMap.get(DcMotor.class, "CWL");
        carouselWheelRight = hardwareMap.get(DcMotor.class, "CWR");

        capper           = hardwareMap.get(CRServo.class, "CAP");
        capper2 = hardwareMap.get(CRServo.class, "CAP2");

        intake = hardwareMap.get(CRServo.class, "Intake");
        guideWheelLeft = hardwareMap.get(CRServo.class, "GWL");
        guideWheelRight = hardwareMap.get(CRServo.class, "GWR");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "CS");

        LED_LRed = hardwareMap.get(DigitalChannel.class, "LED_L-Red");
        LED_LGreen = hardwareMap.get(DigitalChannel.class, "LED_L-Green");

        LED_RRed = hardwareMap.get(DigitalChannel.class, "LED_R-Red");
        LED_RGreen = hardwareMap.get(DigitalChannel.class, "LED_R-Green");

        leftTurretLimit = hardwareMap.get(RevTouchSensor.class, "LTL");
        rightTurretLimit = hardwareMap.get(RevTouchSensor.class, "RTL");
        lowArmHeightLimit = hardwareMap.get(RevTouchSensor.class, "LAHL");

        armMotor.setDirection(DcMotor.Direction.REVERSE);
        guideWheelRight.setDirection(CRServo.Direction.REVERSE);


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


        // TODO: ROADRUNNER TRAJECTORIES

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        Pose2d startPose = new Pose2d(12, -64, Math.toRadians(0));

        drive.setPoseEstimate(startPose); // Red Warehouse


        TrajectorySequence cycle1 = drive.trajectorySequenceBuilder(startPose)

                // Pre-load
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-6, -48), Math.toRadians(110))
                .waitSeconds(0.5)

                // 1st cycle
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(16, -64), Math.toRadians(0))
                .splineTo(new Vector2d(24,-64), Math.toRadians(0))
                .splineTo(new Vector2d(44,-64), Math.toRadians(0))

                .lineTo(new Vector2d(12, -64))
                .splineToConstantHeading(new Vector2d(-6, -48), Math.toRadians(110))

                // 2nd cycle
                .splineToConstantHeading(new Vector2d(12, -64), Math.toRadians(0))
                .splineTo(new Vector2d(24,-64), Math.toRadians(0))
                .splineTo(new Vector2d(44,-64), Math.toRadians(0))

                .lineTo(new Vector2d(12, -64))
                .splineToConstantHeading(new Vector2d(-6, -48), Math.toRadians(110))

                // 3rd cycle
                .splineToConstantHeading(new Vector2d(12, -64), Math.toRadians(0))
                .splineTo(new Vector2d(24,-64), Math.toRadians(0))
                .splineTo(new Vector2d(44,-64), Math.toRadians(0))

                .lineTo(new Vector2d(12, -64))
                .splineToConstantHeading(new Vector2d(-6, -48), Math.toRadians(110))

                // 4th cycle
                .splineToConstantHeading(new Vector2d(12, -64), Math.toRadians(0))
                .splineTo(new Vector2d(24,-64), Math.toRadians(0))
                .splineTo(new Vector2d(28,-64), Math.toRadians(0))
                .splineTo(new Vector2d(46, -58), Math.toRadians(50))

                .setReversed(true)
                .splineTo(new Vector2d(32, -64), Math.toRadians(180))
                .lineTo(new Vector2d(12, -64))
                .splineToConstantHeading(new Vector2d(-6, -48), Math.toRadians(110))

                // 5th cycle
                .splineToConstantHeading(new Vector2d(12, -64), Math.toRadians(0))
                .splineTo(new Vector2d(24,-64), Math.toRadians(0))
                .splineTo(new Vector2d(28,-64), Math.toRadians(0))
                .splineTo(new Vector2d(46, -58), Math.toRadians(50))

                .setReversed(true)
                .splineTo(new Vector2d(32, -64), Math.toRadians(180))
                .lineTo(new Vector2d(12, -64))
                .splineToConstantHeading(new Vector2d(-6, -48), Math.toRadians(110))

                // 6th cycle
                .splineToConstantHeading(new Vector2d(12, -64), Math.toRadians(0))
                .splineTo(new Vector2d(24,-64), Math.toRadians(0))
                .splineTo(new Vector2d(44,-64), Math.toRadians(0))

                .lineTo(new Vector2d(12, -64))
                .splineToConstantHeading(new Vector2d(-6, -48), Math.toRadians(110))

                // Park
                .splineToConstantHeading(new Vector2d(12, -64), Math.toRadians(0))
                .splineTo(new Vector2d(24,-64), Math.toRadians(0))
                .splineTo(new Vector2d(44,-64), Math.toRadians(0))


                .build();




        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Scanner scanner = new Scanner(telemetry);
        webcam.setPipeline(scanner);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode)
            {
            }
        });



        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        Barcode result = scanner.getResult();


        switch (result) {

            case LEFT:

                drive.followTrajectorySequenceAsync(cycle1);


                break;

            case MIDDLE:

                drive.followTrajectorySequenceAsync(cycle1);


                break;

            case RIGHT:

                drive.followTrajectorySequenceAsync(cycle1);


                break;
        }

/*
        while (opModeIsActive() && !isStopRequested()) {

            if (drive.getPoseEstimate().getX()) {
                intakeState = true;

                if (hasCargo && intakeState) {
                    intakeState = false;
                }
            }
            if (hasCargo) {
                intake.setPower(0);
            }
            if (!drive.getPoseEstimate().epsilonEquals(new Pose2d(24, -64)) &&
                    !drive.getPoseEstimate().epsilonEquals(new Pose2d(-6, -48))) {
                intake.setPower(0);
            }


            // Update drive
            drive.update();
        }
 */

    }


    public void moveArm(int targetPosition, double power) {
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(power);
    }

    public void moveTurret(int targetPosition, double power) {
        turret.setTargetPosition(targetPosition);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(power);
    }

}
