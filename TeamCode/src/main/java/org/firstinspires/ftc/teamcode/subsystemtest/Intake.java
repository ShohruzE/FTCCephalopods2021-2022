package org.firstinspires.ftc.teamcode.subsystemtest;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Intake extends Subsystem {

    CRServo intake;
    CRServo guideWheelLeft;
    CRServo guideWheelRight;
    RevColorSensorV3 colorSensor;

    DigitalChannel LED_LRed;
    DigitalChannel LED_LGreen;
    DigitalChannel LED_RRed;
    DigitalChannel LED_RGreen;

    public static double intakeSpeed = 1;


    public Intake(OpMode opMode) {
        super(opMode);
    }


    @Override
    public void init() {

        hardwareMap = opMode.hardwareMap;
        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;

        intake = hardwareMap.get(CRServo.class, "Intake");
        guideWheelLeft = hardwareMap.get(CRServo.class, "GWL");
        guideWheelRight = hardwareMap.get(CRServo.class, "GWR");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "CS");

        LED_LRed = hardwareMap.get(DigitalChannel.class, "LED_L-Red");
        LED_LGreen = hardwareMap.get(DigitalChannel.class, "LED_L-Green");

        LED_RRed = hardwareMap.get(DigitalChannel.class, "LED_R-Red");
        LED_RGreen = hardwareMap.get(DigitalChannel.class, "LED_R-Green");

        LED_LRed.setMode(DigitalChannel.Mode.OUTPUT);
        LED_LGreen.setMode(DigitalChannel.Mode.OUTPUT);
        LED_RRed.setMode(DigitalChannel.Mode.OUTPUT);
        LED_RGreen.setMode(DigitalChannel.Mode.OUTPUT);

        guideWheelRight.setDirection(CRServo.Direction.REVERSE);
        intake.setPower(0);
    }

    @Override
    public void run() {

        boolean hasCargo = colorSensor.getDistance(DistanceUnit.CM) <= 2.5;

        if ((gamepad1.left_trigger > 0.2 || gamepad2.right_trigger > 0.2)) { // Intake && !hasCargo

            intake.setPower(-intakeSpeed); /* Math.Max(gamepad1.rightTrigger, gamepad2.rightTrigger) */
            guideWheelLeft.setPower(-intakeSpeed);
            guideWheelRight.setPower(-intakeSpeed);
        }
        else if (gamepad1.right_trigger > 0.2 || gamepad2.left_trigger > 0.2) { // Fast outtake

            intake.setPower(intakeSpeed);
            guideWheelLeft.setPower(intakeSpeed);
            guideWheelRight.setPower(intakeSpeed);/* -Math.Max(gamepad1.leftTrigger, gamepad2.leftTrigger) */
        }
        else if (gamepad1.right_bumper || gamepad2.right_bumper) { // Slow outtake

            intake.setPower(0.3);
            guideWheelLeft.setPower(0.3);
            guideWheelRight.setPower(0.3);
        }
        else {
            intake.setPower(0);
            guideWheelLeft.setPower(0);
            guideWheelRight.setPower(0);
        }

        if (hasCargo) {
            LED_LRed.setState(true);
            LED_LGreen.setState(false);
            LED_RRed.setState(true);
            LED_RGreen.setState(false);
        }
        else {
            LED_LRed.setState(false);
            LED_LGreen.setState(true);
            LED_RRed.setState(false);
            LED_RGreen.setState(true);
        }
    }
}
