package org.firstinspires.ftc.teamcode.subsystemtest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class SusbystemTeleOp extends OpMode {

    private ElapsedTime gameTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);


    Arm arm = new Arm(this);
    Capper capper = new Capper(this);
    Intake intake = new Intake(this);
    Turret turret = new Turret(this);
    CarouselWheel carouselWheel = new CarouselWheel(this);
    Drive drive = new Drive(this);


    @Override
    public void init() {


        arm.init();
        capper.init();
        intake.init();
        turret.init();
        carouselWheel.init();
        drive.init();

        telemetry.addData("Status", "Initialized");


    }

    @Override
    public void start() {

        gameTime.reset();
    }

    @Override
    public void loop() {


        arm.run();
        capper.run();
        intake.run();
        turret.run();
        carouselWheel.run();
        drive.run();

        telemetry.addData("Status", "Run Time: " + gameTime.toString());
        telemetry.addData("Turret: ", turret.turret.getCurrentPosition());
        telemetry.addData("Arm: ", arm.armMotor.getCurrentPosition());
        telemetry.addData("CarouselWheelLeft: ", carouselWheel.carouselWheelLeft.getCurrentPosition());
        telemetry.addData("CarouselWheelRight: ", carouselWheel.carouselWheelRight.getCurrentPosition());

    }
}
