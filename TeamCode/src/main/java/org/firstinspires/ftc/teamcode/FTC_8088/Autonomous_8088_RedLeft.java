package org.firstinspires.ftc.teamcode.FTC_8088;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Barcode;
import org.firstinspires.ftc.teamcode.Scanner;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Autonomous_RedLeft - Team_8088", group = "Autonomous - 8088")
public class Autonomous_8088_RedLeft extends LinearOpMode {

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
    ColorSensor colorSensor2;

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
    double turretSpeed = 0.4;
    double armMotorSpeed = 0.4;
    double carouselWheelSpeed = 0.6;

    double powerMultiplier = 1.0;


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
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "CS2");

        LED_LRed = hardwareMap.get(DigitalChannel.class, "LED_L-Red");
        LED_LGreen = hardwareMap.get(DigitalChannel.class, "LED_L-Green");

        LED_RRed = hardwareMap.get(DigitalChannel.class, "LED_R-Red");
        LED_RGreen = hardwareMap.get(DigitalChannel.class, "LED_R-Green");

        leftTurretLimit = hardwareMap.get(RevTouchSensor.class, "LTL");
        rightTurretLimit = hardwareMap.get(RevTouchSensor.class, "RTL");
        lowArmHeightLimit = hardwareMap.get(RevTouchSensor.class, "LAHL");

        armMotor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(CRServo.Direction.REVERSE);
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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, -64, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence collectDuck1 = drive.trajectorySequenceBuilder(startPose)

                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> moveArm(400, 0.4))
                .splineToConstantHeading(new Vector2d(-35, -58), Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(-64, -51, Math.toRadians(0)))
                .strafeRight(5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> carouselWheelRight.setPower(-0.6))
                .waitSeconds(3.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> carouselWheelRight.setPower(0))

                //       .splineToConstantHeading(new Vector2d(-47, -47), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(-52, -50, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> moveArm(1200, 0.4))
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> moveTurret(-925, 0.25))
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> moveArm(600, 0.2))
                .splineTo(new Vector2d(-56, -22), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(-32, -20), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(-1)) // Deliver pre-load
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(0))

                /*
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> moveArm(800, 0.4))
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> moveTurret(0, 0.3))
                .UNSTABLE_addTemporalMarkerOffset(2, () -> moveArm(100, 0.3))
                .lineTo(new Vector2d(-56, -24))
                .setReversed(false)
                .splineTo(new Vector2d(-61, -46), Math.toRadians(210))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> moveArm(50, 0.2))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(1))
                .splineToConstantHeading(new Vector2d(-61, -59), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20)
                ) // slow down here
                .splineTo(new Vector2d(-62, -61), Math.toRadians(359)) // angle switches here below carousel

                .splineToConstantHeading(new Vector2d(-48, -61), Math.toRadians(0)) // reaches end of first pass-through
                .turn(Math.toRadians(-60)) // turns to face other direction for second pass-through
                .lineTo(new Vector2d(-62, -61))

                .resetConstraints()
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(0))
                .splineTo(new Vector2d(-56, -24), Math.toRadians(90)) // exits area to prepare delivery
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> moveArm(1200, 0.4))
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> moveTurret(-900, 0.3))
                .splineToConstantHeading(new Vector2d(-36, -24), Math.toRadians(0)) // deliver duck
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(-1))
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(0))

                 */

                .UNSTABLE_addTemporalMarkerOffset(1, () -> moveArm(1200, 0.4))
                .UNSTABLE_addTemporalMarkerOffset(2, () -> moveTurret(0, 0.3))
                .splineToSplineHeading(new Pose2d(-56, -24 , Math.toRadians(0)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-60, -42), Math.toRadians(270)) // park
                .UNSTABLE_addTemporalMarkerOffset(0, () -> moveArm(-50, 0.3))

                .build();

        TrajectorySequence collectDuck2 = drive.trajectorySequenceBuilder(startPose)

                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> moveArm(400, 0.4))
                .splineToConstantHeading(new Vector2d(-35, -58), Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(-64, -51, Math.toRadians(0)))
                .strafeRight(5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> carouselWheelRight.setPower(-0.6))
                .waitSeconds(3.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> carouselWheelRight.setPower(0))

                //       .splineToConstantHeading(new Vector2d(-47, -47), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(-52, -50, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> moveArm(1000, 0.4))
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> moveTurret(-925, 0.25))
                .splineTo(new Vector2d(-56, -24), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(-34, -20), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(-1)) // Deliver pre-load
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(0))

                /*


                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> moveTurret(0, 0.3))
                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> moveArm(100, 0.3))
                .lineTo(new Vector2d(-56, -24))
                .setReversed(false)
                .splineTo(new Vector2d(-61, -46), Math.toRadians(210))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> moveArm(50, 0.2))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(1))
                .splineToConstantHeading(new Vector2d(-61, -59), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20)
                ) // slow down here
                .splineTo(new Vector2d(-62, -61), Math.toRadians(359)) // angle switches here below carousel

                .splineToConstantHeading(new Vector2d(-48, -61), Math.toRadians(0)) // reaches end of first pass-through
                .turn(Math.toRadians(-60)) // turns to face other direction for second pass-through
                .lineTo(new Vector2d(-62, -61))

                .resetConstraints()
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(0))
                .splineTo(new Vector2d(-56, -24), Math.toRadians(90)) // exits area to prepare delivery
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> moveArm(1200, 0.4))
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> moveTurret(-900, 0.3))
                .splineToConstantHeading(new Vector2d(-36, -24), Math.toRadians(0)) // deliver duck
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(-1))
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(0))

                 */

                .UNSTABLE_addTemporalMarkerOffset(1, () -> moveTurret(0, 0.3))
                .UNSTABLE_addTemporalMarkerOffset(2, () -> moveArm(800, 0.2))
                .splineToSplineHeading(new Pose2d(-56, -24 , Math.toRadians(0)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-60, -42), Math.toRadians(270)) // park
                .UNSTABLE_addTemporalMarkerOffset(0, () -> moveArm(-50, 0.2))

                .build();

        TrajectorySequence collectDuck3 = drive.trajectorySequenceBuilder(startPose)

                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> moveArm(800, 0.4))
                .splineToConstantHeading(new Vector2d(-35, -58), Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(-64, -51, Math.toRadians(0)))
                .strafeRight(5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> carouselWheelRight.setPower(-0.6))
                .waitSeconds(3.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> carouselWheelRight.setPower(0))

                //       .splineToConstantHeading(new Vector2d(-47, -47), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(-52, -50, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> moveArm(1600, 0.4))
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> moveTurret(-925, 0.25))
                .splineTo(new Vector2d(-56, -24), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(-30, -20), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(-1)) // Deliver pre-load
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(0))

                /*

                .UNSTABLE_addTemporalMarkerOffset(0, () -> moveArm(800, 0.4))
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> moveTurret(0, 0.3))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> moveArm(100, 0.3))
                .lineTo(new Vector2d(-56, -24))
                .setReversed(false)
                .splineTo(new Vector2d(-61, -46), Math.toRadians(210))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> moveArm(50, 0.2))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(1))
                .splineToConstantHeading(new Vector2d(-61, -59), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20)
                ) // slow down here
                .splineTo(new Vector2d(-62, -61), Math.toRadians(359)) // angle switches here below carousel

                .splineToConstantHeading(new Vector2d(-48, -61), Math.toRadians(0)) // reaches end of first pass-through
                .turn(Math.toRadians(-60)) // turns to face other direction for second pass-through
                .lineTo(new Vector2d(-62, -61))

                .resetConstraints()
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(0))
                .splineTo(new Vector2d(-56, -24), Math.toRadians(90)) // exits area to prepare delivery
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> moveArm(1200, 0.4))
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> moveTurret(-900, 0.3))
                .splineToConstantHeading(new Vector2d(-36, -24), Math.toRadians(0)) // deliver duck
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(-1))
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(0))


                 */

                .UNSTABLE_addTemporalMarkerOffset(0, () -> moveTurret(0, 0.3))
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> moveArm(900, 0.2))
                .splineToSplineHeading(new Pose2d(-56, -24 , Math.toRadians(0)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-60, -42), Math.toRadians(270)) // park
                .UNSTABLE_addTemporalMarkerOffset(0, () -> moveArm(-50, 0.2))

                .build();


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();


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


        waitForStart();// TODO: PLACE AUTONOMOUS CODE AFTER THIS LINE
        runtime.reset();

        if (isStopRequested()) return;

        Barcode result = scanner.getResult();


        switch (result) {

            case LEFT:

                drive.followTrajectorySequence(collectDuck1);


                break;

            case MIDDLE:

                drive.followTrajectorySequence(collectDuck2);


                break;

            case RIGHT:

                drive.followTrajectorySequence(collectDuck3);


                break;
        }
    }


    // ROBOT AUTONOMOUS FUNCTIONS

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
