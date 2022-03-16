package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private double targetAngle;
    private double kP;
    private double kI;
    private double kD;

    private double accumulatedError = 0;
    private double lastTime = 0;
    private double lastError = 0;

    private ElapsedTime runtime = new ElapsedTime();

    // Constructor for stating PID coefficients
    public PIDController(double target, double p, double i, double d) {
        targetAngle = target;
        kP = p;
        kI = i;
        kD = d;
    }

    public double update(double currentAngle) {

        // P
        double error = targetAngle - currentAngle;

        error %= 360;
        error += 360;
        error %= 360;

        if (error > 180) {
            error -= 360;
        }


        // I
        accumulatedError += error;
        if (Math.abs(error) < 1) {
            accumulatedError = 0;
        }
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error);


        // D
        double slope = 0;
        if (lastTime > 0) {

            slope = (error - lastError) / (runtime.milliseconds() - lastTime);
        }
        lastTime = runtime.milliseconds();
        lastError = error;


        // Ensures motorPower is within -1 and 1;
        double motorPower = 0.1 * Math.signum(error) + 0.9 * Math.tanh(kP * error + kI *
                                                                accumulatedError + kD * slope);

        return motorPower;
    }
}
