package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class MeepMeepTestingBlueRight {

    public static void main(String args[]) {

        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep mm = new MeepMeep(800)
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(1f)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11.5)
                .setBotDimensions(13,17)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(new Pose2d(-35, 64, Math.toRadians(0)))

                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-35, 58), Math.toRadians(-90))
                                .lineToLinearHeading(new Pose2d(-54, 60, Math.toRadians(50)))
                                .waitSeconds(3.5)

                                //       .splineToConstantHeading(new Vector2d(-47, -47), Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(-52, 50, Math.toRadians(-270)))
                                .splineTo(new Vector2d(-56, 24), Math.toRadians(-90))

                                .splineToConstantHeading(new Vector2d(-36, 24), Math.toRadians(0))
                                .waitSeconds(1)

                                .lineTo(new Vector2d(-56, 24))
                                .setReversed(false)
                                .splineTo(new Vector2d(-61, 46), Math.toRadians(-210))

                                .splineToConstantHeading(new Vector2d(-61, 59), Math.toRadians(-270)) // slow down here
                                .splineTo(new Vector2d(-62, 61), Math.toRadians(-359)) // angle switches here below carousel

                                .splineToConstantHeading(new Vector2d(-48, 61), Math.toRadians(0)) // reaches end of first pass-through
                                .turn(Math.toRadians(60)) // turns to face other direction for second pass-through
                                .lineTo(new Vector2d(-62, 61))

                                .setReversed(true)
                                .splineTo(new Vector2d(-56, 24), Math.toRadians(-90)) // exits area to prepare delivery
                                .splineToConstantHeading(new Vector2d(-36, 24), Math.toRadians(0)) // deliver duck

                                .splineToSplineHeading(new Pose2d(-56, 24 ,Math.toRadians(0)), Math.toRadians(-180))
                                .splineToConstantHeading(new Vector2d(-60, 35), Math.toRadians(-270)) // park




                                        /*
                                        .lineToLinearHeading(new Pose2d(-62, 50, Math.toRadians(0)))
                                        .strafeLeft(5)
                                        .waitSeconds(3)
                                        .lineToLinearHeading(new Pose2d(-53, 24, Math.toRadians(90)))
                                        .strafeTo(new Vector2d(-28, 24))
                                        .waitSeconds(1)
                                        .lineToLinearHeading(new Pose2d(-62, 24, Math.toRadians(0)))
                                        .strafeLeft(12)
                                         */

                                        .build()
                )
                .start();
    }
}
