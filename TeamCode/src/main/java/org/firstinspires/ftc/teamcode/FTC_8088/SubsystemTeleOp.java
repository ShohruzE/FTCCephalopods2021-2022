package org.firstinspires.ftc.teamcode.FTC_8088;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystemtest.Arm;
import org.firstinspires.ftc.teamcode.subsystemtest.Capper;
import org.firstinspires.ftc.teamcode.subsystemtest.Drive;
import org.firstinspires.ftc.teamcode.subsystemtest.Intake;
import org.firstinspires.ftc.teamcode.subsystemtest.Turret;


@TeleOp
public class SubsystemTeleOp extends OpMode {
    Arm arm = new Arm(this);
    Capper capper = new Capper(this);
    Drive drive = new Drive(this);
    Intake intake = new Intake(this);
    Turret turret = new Turret(this);


    @Override
    public void init() {
        arm.init();
        capper.init();
        drive.init();
        intake.init();
        turret.init();
    }

    @Override
    public void loop() {
        arm.run();
        capper.run();
        drive.run();
        intake.run();
        turret.run();
    }
}
