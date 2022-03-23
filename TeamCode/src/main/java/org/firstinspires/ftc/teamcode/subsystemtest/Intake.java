package org.firstinspires.ftc.teamcode.subsystemtest;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Intake extends Subsystem {

    CRServo intake;
    RevColorSensorV3 colorSensor;

    public static double intakeSpeed = 1;


    public Intake(OpMode opMode) {
        super(opMode);
    }


    @Override
    public void init() {

        hardwareMap = opMode.hardwareMap;

        intake = hardwareMap.get(CRServo.class, "Intake");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "CS");
        intake.setPower(0);
    }

    @Override
    public void run() {

        boolean hasCargo = colorSensor.getDistance(DistanceUnit.CM) <= 3.5;

        if (gamepad1.right_trigger > .2 || gamepad2.right_trigger > .2 && !hasCargo) {
            intake.setPower(intakeSpeed); /* Math.Max(gamepad1.rightTrigger, gamepad2.rightTrigger) */
        } else if (gamepad1.left_trigger > .2 || gamepad2.left_trigger > .2) {
            intake.setPower(-intakeSpeed); /* -Math.Max(gamepad1.leftTrigger, gamepad2.leftTrigger) */
        } else {
            intake.setPower(0);
        }
    }
}
