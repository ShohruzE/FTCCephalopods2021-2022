package org.firstinspires.ftc.teamcode.subsystemtest;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Intake extends Subsystem {

    CRServo intake;
    CRServo guideWheelLeft;
    CRServo guideWheelRight;
    RevColorSensorV3 colorSensor;

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
        intake.setPower(0);
    }

    @Override
    public void run() {

        boolean hasCargo = colorSensor.getDistance(DistanceUnit.CM) <= 3.5;

        // TODO: Add slower intake and outtake speeds after testing if necessary
        if (gamepad1.left_trigger > .2 || gamepad2.left_trigger > .2 && !hasCargo) {

            intake.setPower(intakeSpeed); /* Math.Max(gamepad1.rightTrigger, gamepad2.rightTrigger) */
            guideWheelLeft.setPower(intakeSpeed);
            guideWheelRight.setPower(-intakeSpeed);
        }
        else if (gamepad1.right_trigger > .2 || gamepad2.right_trigger > .2) {

            intake.setPower(-intakeSpeed); /* -Math.Max(gamepad1.leftTrigger, gamepad2.leftTrigger) */
            guideWheelLeft.setPower(-intakeSpeed);
            guideWheelRight.setPower(intakeSpeed);
        }
        else {
            intake.setPower(0);
            guideWheelLeft.setPower(0);
            guideWheelRight.setPower(0);
        }
    }
}
