package com.example.teamcode_8087

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware_9385{

    // Drivetrain Motors
    public DcMotor frontLeft  = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft   = null;
    public DcMotor backRight  = null;

    // Other Motors


    // Servos
    public Servo somethingServo = null;

    // Other Variables
    HardwareMap hardwareMap = null;
    public ElapsedTime runTime = new ElapsedTime();


    public Hardware_9385(HardwareMap map) {

        init(map);
    }

    public void init(HardwareMap map) {

        hardwareMap = map;

        // Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft   = hardwareMap.get(DcMotor.class, "bottomLeftMotor");
        backRight  = hardwareMap.get(DcMotor.class, "bottomRightMotor");

        // Servos


        // Set motor direction
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Set ZERO POWER BEHAVIOR
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

