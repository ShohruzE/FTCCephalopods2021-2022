package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class MeepMeepTestingRedRight {

    public static void main(String args[]) {

        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep mm = new MeepMeep(800)
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(1f)
                .setConstraints(60, 50, Math.toRadians(180), Math.toRadians(180), 12)
                .setBotDimensions(13,17.5)
                .followTrajectorySequence(drive ->

                                drive.trajectorySequenceBuilder(new Pose2d(12,-64, Math.toRadians(0)))

                                        // Pre-load
                                        .setReversed(true)
                                        .splineTo(new Vector2d(0,-38), Math.toRadians(45))

                                        // 1st cylce
                                        .setReversed(false)
                                        .splineTo(new Vector2d(24,-64), Math.toRadians(0))
                                        .splineTo(new Vector2d(44,-64), Math.toRadians(0))

                                        .setReversed(true)
                                        .lineTo(new Vector2d(12, -64))
                                        .splineTo(new Vector2d(0,-38), Math.toRadians(45))

                                        // 2nd cycle
                                        .setReversed(false)
                                        .splineTo(new Vector2d(24,-64), Math.toRadians(0))
                                        .splineTo(new Vector2d(44,-64), Math.toRadians(0))

                                        .setReversed(true)
                                        .lineTo(new Vector2d(12, -64))
                                        .splineTo(new Vector2d(0,-38), Math.toRadians(45))

                                        // 3rd cycle
                                        .setReversed(false)
                                        .splineTo(new Vector2d(24,-64), Math.toRadians(0))
                                        .splineTo(new Vector2d(38,-64), Math.toRadians(0))
                                        .splineTo(new Vector2d(46, -58), Math.toRadians(50))    // Two splines


                                        .setReversed(true)
                                        .splineTo(new Vector2d(38, -64), Math.toRadians(180))
                                        .lineTo(new Vector2d(12, -64))
                                        .splineTo(new Vector2d(0,-38), Math.toRadians(45))

                                        // 4th cycle
                                        .setReversed(false)
                                        .splineTo(new Vector2d(24,-64), Math.toRadians(0))
                                        .splineTo(new Vector2d(44,-64), Math.toRadians(0))

                                        .setReversed(true)
                                        .lineTo(new Vector2d(12, -64))
                                        .splineTo(new Vector2d(0,-38), Math.toRadians(45))

                                        // Park
                                        .setReversed(false)
                                        .splineTo(new Vector2d(24,-64), Math.toRadians(0))
                                        .splineTo(new Vector2d(44,-64), Math.toRadians(0))


                        //                .splineTo(new Vector2d(12,-63), Math.toRadians(0))
                        //                .splineTo(new Vector2d(20, -64), Math.toRadians(0))
                        //                .splineTo(new Vector2d(44,-64), Math.toRadians(0))

                                        /*
                                        .splineToSplineHeading(new Pose2d(0, -38, Math.toRadians(45)), Math.toRadians(45))

                                        .splineToSplineHeading(new Pose2d(10, -55, Math.toRadians(0)), Math.toRadians(270))
                                        .splineToConstantHeading(new Vector2d(44, -65), Math.toRadians(355))


                                        .splineToConstantHeading(new Vector2d(6, -64), Math.toRadians(355))
                                        // fix weird hook in spline below to mimic first spline
                                        .splineToSplineHeading(new Pose2d(0, -38, Math.toRadians(45)), Math.toRadians(45))
                                        */




                                        /*

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


                                         */


                                .build()
                )
                .start();
    }
}