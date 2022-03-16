package org.firstinspires.ftc.teamcode.FTC_9385;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.logging.Level;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

@Disabled
@Autonomous(name = "Blocks_Auto_RedWarehouse_Enc (Blocks to Java)")
public class Autonomous_KaitlynDad extends LinearOpMode {

    private DcMotor FL;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor BR;
    private CRServo Intake;
    private CRServo Cap;
    private DcMotor CWL;
    private DcMotor CWR;
    private DcMotor Arm;
    private VuforiaCurrentGame vuforiaFreightFrenzy;
    private TfodCurrentGame tfodFreightFrenzy;

    int FLPos;
    int BLPos;
    int FRPos;
    int BRPos;
    Recognition recognition;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        List<Recognition> recognitions;
        int index;
        int Level;

        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        Intake = hardwareMap.get(CRServo.class, "Intake");
        Cap = hardwareMap.get(CRServo.class, "Cap");
        CWL = hardwareMap.get(DcMotor.class, "CWL");
        CWR = hardwareMap.get(DcMotor.class, "CWR");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        vuforiaFreightFrenzy = new VuforiaCurrentGame();
        tfodFreightFrenzy = new TfodCurrentGame();

        // Put initialization blocks here.
        telemetry.addData("Autonomous:", "Red Team - Park at Warehouse");
        telemetry.update();
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLPos = 0;
        BLPos = 0;
        FRPos = 0;
        BRPos = 0;
        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        Cap.setDirection(DcMotorSimple.Direction.REVERSE);
        CWL.setDirection(DcMotorSimple.Direction.REVERSE);
        CWR.setDirection(DcMotorSimple.Direction.FORWARD);
        Arm.setDirection(DcMotorSimple.Direction.FORWARD);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Initialize Vuforia.
        // We need Vuforia to provide TFOD with camera images.
        vuforiaFreightFrenzy.initialize(
                "", // vuforiaLicenseKey
                VuforiaLocalizer.CameraDirection.BACK, // cameraDirection
                true, // useExtendedTracking
                false, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                AxesOrder.XYZ, // axesOrder
                0, // firstAngle
                -90, // secondAngle
                0, // thirdAngle
                true); // useCompetitionFieldTargetLocations
        // Set min confidence threshold to 0.7
        tfodFreightFrenzy.initialize(vuforiaFreightFrenzy, (float) 0.7, true, true);
        // Initialize TFOD before waitForStart.
        // Init TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfodFreightFrenzy.activate();
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            // 1 - Detect Ducky Location (TBA)
            // Get a list of recognitions from TFOD.
            recognitions = tfodFreightFrenzy.getRecognitions();
            // If list is empty, inform the user. Otherwise, go
            // through list and display info for each recognition.
            if (recognitions.size() == 0) {
                telemetry.addData("TFOD", "No items detected.");
            } else {
                index = 0;
                // Iterate through list and call a function to
                // display info for each recognized object.
                for (Recognition recognition_item : recognitions) {
                    recognition = recognition_item;
                    // Display info.
                    displayInfo(index);
                    // Increment index.
                    index = index + 1;
                }
            }
            /* TODO: Commented lines below are for TFOD marker detection
            telemetry.addData("left", recognitions.getLeft());
            if (recognitions.getLeft() > 900) {
                Level = 3;
            } else if (recognitions.getLeft() > 500) {
                Level = 2;
            } else {
                Level = 1;
            }
            telemetry.addData("Level", Level);
            telemetry.update();
            TODO: Commented lines above are for TFOD marker detection
            */

            // Deactivate TFOD.
            tfodFreightFrenzy.deactivate();
            // 2 - Move Backward
            Drive(-2000, -2000, -2000, -2000, 1);
            Reset_Motor_Pos();
            // 3 - Turn Clockwise 90
            Drive(2500, 2500, -2500, -2500, 1);
            // 4 - Lift Arm and Hold (TBA for More Code)
            sleep(3000);
            // 5 - Move forward
            Drive(1834, 1263, -3986, -3666, 1);
            // 6 - Drop Cube
            // 7 - Move Backward
            Drive(1276, 766, -4409, -4139, 1);
            // 8 - Lower Arm
            // 9-Turn
            Drive(156, -44, -3487, -3428, 1);
            // 10 - Move Backward
            Drive(-6638, -6737, -9944, -9915, 1);
            // 11 - Lower Arm
        }

        vuforiaFreightFrenzy.close();
        tfodFreightFrenzy.close();
    }

    /**
     * Describe this function...
     */
    private void Drive(int FLTarget, int BLTarget, int FRTarget, int BRTarget, int Speed) {
        FLPos += FLTarget;
        BLPos += BLTarget;
        FRPos += FRTarget;
        BRPos += BRTarget;
        FL.setTargetPosition(FLPos);
        BL.setTargetPosition(BLPos);
        FR.setTargetPosition(FRPos);
        BR.setTargetPosition(BRPos);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setPower(Speed);
        BL.setPower(Speed);
        FR.setPower(Speed);
        BR.setPower(Speed);
        while (opModeIsActive() && FL.isBusy() && BL.isBusy() && FR.isBusy() && BR.isBusy()) {
            idle();
        }
    }

    /**
     * Describe this function...
     */
    private void Reset_Motor_Pos() {
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLPos = 0;
        BLPos = 0;
        FRPos = 0;
        BRPos = 0;
    }

    /**
     * Display info (using telemetry) for a recognized object.
     */
    private void displayInfo(int i) {
        // Display label info.
        // Display the label and index number for the recognition.
        telemetry.addData("label " + i, recognition.getLabel());
        // Display upper corner info.
        // Display the location of the top left corner
        // of the detection boundary for the recognition
        telemetry.addData("Left, Top " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)));
        // Display lower corner info.
        // Display the location of the bottom right corner
        // of the detection boundary for the recognition
        telemetry.addData("Right, Bottom " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getBottom(), 0)));
    }
}
