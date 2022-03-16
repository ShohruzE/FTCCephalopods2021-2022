package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "WebcamAutoTest", group = "Autonomous - 8087")
public class WebcamAutoTest extends LinearOpMode {

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
    double carouselWheelSpeed = 0.6;

    double powerMultiplier = 1.0;

    private ElapsedTime runTime = new ElapsedTime();


    OpenCvWebcam webcam;
    HardwareMap hardwareMap;
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        
        Pose2d startPose = new Pose2d(-35, 64, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory storagePark = drive.trajectoryBuilder(startPose)

                .lineToLinearHeading(new Pose2d(-62, 50, Math.toRadians(0)))
                .strafeLeft(5)
                // add marker
                .lineToLinearHeading(new Pose2d(-53, 24, Math.toRadians(90)))
                .strafeTo(new Vector2d(-28, 24))
                // add marker
                .lineToLinearHeading(new Pose2d(-62, 24, Math.toRadians(0)))
                .strafeLeft(12)
                .build();


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runTime.toString());
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

        waitForStart();

        runTime.reset();

        Barcode result = scanner.getResult();



        switch (result) {
            case LEFT:


                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setTargetPosition(750);
                armMotor.setPower(armMotorSpeed);


                break;
            case MIDDLE:


                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setTargetPosition(1500);
                armMotor.setPower(armMotorSpeed);

                break;
            case RIGHT:


                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setTargetPosition(2500);
                armMotor.setPower(armMotorSpeed);

                break;


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