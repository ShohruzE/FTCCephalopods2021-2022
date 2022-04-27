package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;

public class MeepMeepTestingBlueLeft {

    public static void main(String args[]) {

        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep mm = new MeepMeep(800)
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(1f)
                .setConstraints(50, 40, Math.toRadians(180), Math.toRadians(180), 12)
                .setBotDimensions(13,17.5)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(new Pose2d(12,64, Math.toRadians(0)))

                                // Pre-load

                                .splineTo(new Vector2d(42, 65), Math.toRadians(0))
                                .splineTo(new Vector2d(48,63), Math.toRadians(-25))

                                .splineTo(new Vector2d(44, 65), Math.toRadians(0))
                                .lineTo(new Vector2d(12, 65))
                                .setReversed(false)


                                /*
                                .setReversed(true)
                                .splineTo(new Vector2d(0,38), Math.toRadians(315))

                                // 1st cylce
                                .setReversed(false)
                                .splineTo(new Vector2d(24,64), Math.toRadians(0))
                                .splineTo(new Vector2d(44,64), Math.toRadians(0))

                                .setReversed(true)
                                .lineTo(new Vector2d(12, 64))
                                .splineTo(new Vector2d(0,38), Math.toRadians(315))

                                // 2nd cycle
                                .setReversed(false)
                                .splineTo(new Vector2d(24,64), Math.toRadians(0))
                                .splineTo(new Vector2d(44,64), Math.toRadians(0))

                                .setReversed(true)
                                .lineTo(new Vector2d(12, 64))
                                .splineTo(new Vector2d(0,38), Math.toRadians(315))

                                // 3rd cycle
                                .setReversed(false)
                                .splineTo(new Vector2d(24,64), Math.toRadians(0))
                                .splineTo(new Vector2d(38,64), Math.toRadians(0))
                                .splineTo(new Vector2d(46, 58), Math.toRadians(310))    // Two splines


                                .setReversed(true)
                                .splineTo(new Vector2d(38, 64), Math.toRadians(180))
                                .lineTo(new Vector2d(12, 64))
                                .splineTo(new Vector2d(0,38), Math.toRadians(315))

                                // 4th cycle
                                .setReversed(false)
                                .splineTo(new Vector2d(24,64), Math.toRadians(0))
                                .splineTo(new Vector2d(44,64), Math.toRadians(0))

                                .setReversed(true)
                                .lineTo(new Vector2d(12, 64))
                                .splineTo(new Vector2d(0,38), Math.toRadians(315))

                                // Park
                                .setReversed(false)
                                .splineTo(new Vector2d(24,64), Math.toRadians(0))
                                .splineTo(new Vector2d(44,64), Math.toRadians(0))



                                 */








                                        /*

                                        .lineToLinearHeading(new Pose2d(2,32, Math.toRadians(-45)))
                                        .waitSeconds(1.5)
                                        .lineToLinearHeading(new Pose2d(8,64,Math.toRadians(0)))

                                        .forward(32)
                                        .lineTo(new Vector2d(8, 64))
                                        .lineToLinearHeading(new Pose2d(2, 32, Math.toRadians(-45)))
                                        .waitSeconds(1)
                                        .lineToLinearHeading(new Pose2d(8, 64, Math.toRadians(0)))

                                        .forward(32)
                                        .lineTo(new Vector2d(8, 64))
                                        .lineToLinearHeading(new Pose2d(2, 32, Math.toRadians(-45)))
                                        .waitSeconds(1)
                                        .lineToLinearHeading(new Pose2d(8, 64, Math.toRadians(0)))

                                        .forward(32)
                                        .lineTo(new Vector2d(8, 64))
                                        .lineToLinearHeading(new Pose2d(2, 32, Math.toRadians(-45)))
                                        .waitSeconds(1)
                                        .lineToLinearHeading(new Pose2d(8, 64, Math.toRadians(0)))

                                        .forward(30)
                                        .strafeRight(25)
                                        .splineToLinearHeading(new Pose2d(68, 35, Math.toRadians(90)), Math.toRadians(0))

                                        */

                                        .build()
                )
                .start();
    }
}
