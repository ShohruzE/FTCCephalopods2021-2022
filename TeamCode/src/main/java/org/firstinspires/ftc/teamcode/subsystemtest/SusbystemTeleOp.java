package org.firstinspires.ftc.teamcode.subsystemtest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class SusbystemTeleOp extends OpMode {

    private ElapsedTime gameTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    /*
    Arm arm = new Arm(this);
    Capper capper = new Capper(this);
    Intake intake = new Intake(this);
    Turret turret = new Turret(this);
    */
    Drive drive = new Drive(this);


    @Override
    public void init() {

        /*
        arm.init();
        capper.init();
        intake.init();
        turret.init();
        */
        drive.init();


    }

    @Override
    public void start() {

        gameTime.reset();
    }

    @Override
    public void loop() {

        /*
        arm.run();
        capper.run();
        intake.run();
        turret.run();
        */
        drive.run();
    }
}
