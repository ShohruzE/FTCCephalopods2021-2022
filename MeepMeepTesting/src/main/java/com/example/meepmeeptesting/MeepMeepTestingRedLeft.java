package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class MeepMeepTestingRedLeft { // TODO: 8087 8087 8087 8087 8087 8087 8087

    public static void main(String args[]) {

        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep mm = new MeepMeep(800)
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(1f)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11.5)
                .setBotDimensions(13, 17)
                .followTrajectorySequence(drive ->

                                drive.trajectorySequenceBuilder(new Pose2d(-35, -64, Math.toRadians(0)))

                                        /* TODO: Storage Unit Park
                                        .lineToLinearHeading(new Pose2d(-62, -50, Math.toRadians(0)))
                                        .strafeRight(5)
                                        .waitSeconds(3)
                                        .lineToLinearHeading(new Pose2d(-53, -24, Math.toRadians(270)))
                                        .strafeTo(new Vector2d(-28, -24))
                                        .waitSeconds(1)
                                        .lineToLinearHeading(new Pose2d(-62, -24, Math.toRadians(0)))
                                        .strafeRight(12)
                                        */

                                        // TODO: Warehouse Park

                                        .lineToLinearHeading(new Pose2d(-62, -50, Math.toRadians(0)))
                                        .strafeRight(5)
                                        .waitSeconds(3)
                                        .lineToLinearHeading(new Pose2d(-53, -24, Math.toRadians(270)))
                                        .strafeTo(new Vector2d(-28, -24))
                                        .waitSeconds(1)
                                        .lineToLinearHeading(new Pose2d(-55, -24, Math.toRadians(0)))
                                        .lineTo(new Vector2d(-54, -64))
                                        .waitSeconds(15)
                                        .lineTo(new Vector2d(38, -64))


                           .build()
                )
                .start();
    }
}