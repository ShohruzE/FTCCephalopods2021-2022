package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class MeepMeepTestingRedRight { // TODO: 8087 8087 8087 8087 8087 8087 8087

    public static void main(String args[]) {

        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep mm = new MeepMeep(800)
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(1f)
                .setConstraints(50, 40, Math.toRadians(180), Math.toRadians(180), 13.5)
                .setBotDimensions(13,14)
                .followTrajectorySequence(drive ->

                                drive.trajectorySequenceBuilder(new Pose2d(12,-64, Math.toRadians(0)))

                                        .lineToLinearHeading(new Pose2d(2, -32, Math.toRadians(45)))
                                        .waitSeconds(1)
                                        .lineToLinearHeading(new Pose2d(8,-64,Math.toRadians(0)))

                                        .forward(30)
                                        .splineTo(new Vector2d(43, -64), Math.toRadians(0))
                                        .lineTo(new Vector2d(8, -64))
                                        .lineToLinearHeading(new Pose2d(2, -32, Math.toRadians(45)))
                                        .waitSeconds(1)
                                        .lineToLinearHeading(new Pose2d(8, -64, Math.toRadians(0)))

                                        .forward(32)
                                        .lineTo(new Vector2d(8, -64))
                                        .lineToLinearHeading(new Pose2d(2, -32, Math.toRadians(45)))
                                        .waitSeconds(1)
                                        .lineToLinearHeading(new Pose2d(8, -64, Math.toRadians(0)))


                                        .forward(30)
                                        .strafeLeft(24)
                                        .splineToLinearHeading(new Pose2d(63, -38, Math.toRadians(270)), Math.toRadians(0))


/*                                .splineTo(new Vector2d(2, -37), Math.toRadians(135))
                                .waitSeconds(2)
                                // Move back slightly to allow space for turning to face warehouse
                                .back(8)
                                .turn(Math.toRadians(-45))
                                .turn(Math.toRadians(-90))
                                // Move to the barrier
                                .strafeRight(20)
                                // Move forward to take freight
                                .forward(28)
*/

                                .build()
                )
                .start();
    }
}