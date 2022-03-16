package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware {

    // Motors
    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;

    public DcMotorEx armMotor;
    public DcMotorEx turret;
    public DcMotorEx carouselWheelLeft;
    public DcMotorEx carouselWheelRight;

    public CRServo intake;
    public CRServo capper;

    //   ColorSensor colorSensor;
    public RevTouchSensor leftTurretLimit;
    public RevTouchSensor rightTurretLimit;
    public RevTouchSensor maxArmHeightLimit;

    // Motor Speed
    double frontLeftMotorSpeed;
    double frontRightMotorSpeed;
    double backLeftMotorSpeed;
    double backRightMotorSpeed;

    double intakeSpeed = 1;
    double turretSpeed = 0.4;
    double armMotorSpeed = 0.4;
    double carouselWheelSpeed = 0.6;

    // Other Variables
    HardwareMap hardwareMap = null;

    private ElapsedTime runtime = new ElapsedTime();

    public Hardware() {

    }


    public void init(HardwareMap hwMap) {

        hardwareMap = hwMap;


        frontLeftMotor   = hardwareMap.get(DcMotorEx.class, "FL");
        frontRightMotor  = hardwareMap.get(DcMotorEx.class, "FR");
        backLeftMotor    = hardwareMap.get(DcMotorEx.class, "BL");
        backRightMotor   = hardwareMap.get(DcMotorEx.class, "BR");

        turret           = hardwareMap.get(DcMotorEx.class, "TR");
        armMotor         = hardwareMap.get(DcMotorEx.class, "ARM");
        carouselWheelLeft    = hardwareMap.get(DcMotorEx.class, "CWL");
        carouselWheelRight = hardwareMap.get(DcMotorEx.class, "CWR");

        capper           = hardwareMap.get(CRServo.class, "CAP");
        intake           = hardwareMap.get(CRServo.class, "Intake");

        //      colorSensor      = hardwareMap.get(ColorSensor.class, "color");
        leftTurretLimit = hardwareMap.get(RevTouchSensor.class, "LTL");
        rightTurretLimit = hardwareMap.get(RevTouchSensor.class, "RTL");
        maxArmHeightLimit = hardwareMap.get(RevTouchSensor.class, "MAHL");



        // Set motor direction
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);


        // Set ZERO POWER BEHAVIOR
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    }
}
