package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorManager;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double wait = 0.5;

//        RoadRunnerBotEntity smooth = new DefaultBotBuilder(meepMeep)
//                .setConstraints(60, 60, Math.toRadians(300), Math.toRadians(280), 14)
//                .setColorScheme(new ColorSchemeRedLight())
//                .followTrajectorySequence(drive ->
//                                drive.trajectorySequenceBuilder(new Pose2d(60, 36, Math.toRadians(-180)))
//
//                                        //start loop
//                                        .splineTo(new Vector2d(11.5, 50), Math.toRadians(90))
//                                        .splineTo(new Vector2d(11.5, 60), Math.toRadians(90))
//
//                                        .setReversed(true)
//
//                                        .splineTo(new Vector2d(11.5, -16), Math.toRadians(-90))
//                                        .splineTo(new Vector2d(35, -46), Math.toRadians(-90))
//
//
//                                        .waitSeconds(wait)
//
////                                        //start first cycle
////                                        .setReversed(false)
////
////                                        .splineTo(new Vector2d(11.5, -16), Math.toRadians(90))
////                                        .splineTo(new Vector2d(11.5, 26), Math.toRadians(90))
////
////                                        .setReversed(true)
////
////                                        .splineTo(new Vector2d(11.5, -16), Math.toRadians(-90))
////                                        .splineTo(new Vector2d(35, -46), Math.toRadians(-90))
////
////                                        .waitSeconds(wait)
////
////                                        //start second cycle
////                                        .setReversed(false)
////
////                                        .splineTo(new Vector2d(11.5, -16), Math.toRadians(90))
////                                        .splineTo(new Vector2d(11.5, 26), Math.toRadians(90))
////
////                                        .setReversed(true)
////
////                                        .splineTo(new Vector2d(11.5, -16), Math.toRadians(-90))
////                                        .splineTo(new Vector2d(35, -46), Math.toRadians(-90))
////
////                                        .waitSeconds(wait)
////
////                                        //start third cycle
////                                        .setReversed(false)
////
////                                        .splineTo(new Vector2d(11.5, -16), Math.toRadians(90))
////                                        .splineTo(new Vector2d(11.5, 26), Math.toRadians(90))
////
////                                        .setReversed(true)
////
////                                        .splineTo(new Vector2d(11.5, -16), Math.toRadians(-90))
////                                        .splineTo(new Vector2d(35, -46), Math.toRadians(-90))
//
//                                        .build()
//                );

        RoadRunnerBotEntity smooth = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 70, Math.toRadians(300), Math.toRadians(280), 14)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(60, 36, Math.toRadians(-180)))

                                //start loop
                                .splineTo(new Vector2d(11.5, 50), Math.toRadians(90))
                                .splineTo(new Vector2d(11.5, 60), Math.toRadians(90))

                                .setReversed(true)

                                .splineTo(new Vector2d(11.5, -16), Math.toRadians(-90))
                                .splineTo(new Vector2d(35, -46), Math.toRadians(-90))


                                .waitSeconds(wait)

                                //start first cycle
                                .setReversed(false)

                                .splineTo(new Vector2d(11.5, -16), Math.toRadians(90))
                                .splineTo(new Vector2d(11.5, 60), Math.toRadians(90))

                                .setReversed(true)

                                .splineTo(new Vector2d(11.5, -16), Math.toRadians(-90))
                                .splineTo(new Vector2d(35, -46), Math.toRadians(-90))

                                .waitSeconds(wait)

                                //start second cycle
                                .setReversed(false)

                                .splineTo(new Vector2d(11.5, -16), Math.toRadians(90))
                                .splineTo(new Vector2d(11.5, 60), Math.toRadians(90))

                                .setReversed(true)

                                .splineTo(new Vector2d(11.5, -16), Math.toRadians(-90))
                                .splineTo(new Vector2d(35, -46), Math.toRadians(-90))

                                .waitSeconds(wait)

                                //start third cycle
                                .setReversed(false)

                                .splineTo(new Vector2d(11.5, -16), Math.toRadians(90))
                                .splineTo(new Vector2d(11.5, 60), Math.toRadians(90))

                                .setReversed(true)

                                .splineTo(new Vector2d(11.5, -16), Math.toRadians(-90))
                                .splineTo(new Vector2d(35, -46), Math.toRadians(-90))

                                .build()
                );

        RoadRunnerBotEntity fast = new DefaultBotBuilder(meepMeep)
                .setConstraints(80, 80, Math.toRadians(300), Math.toRadians(280), 14)
                .setColorScheme(new ColorSchemeBlueLight())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(60, 36, Math.toRadians(-180)))

                                //start loop
                                .splineTo(new Vector2d(11.5, 50), Math.toRadians(90))
                                .splineTo(new Vector2d(11.5, 60), Math.toRadians(90))

                                .setReversed(true)

                                .splineToConstantHeading(new Vector2d(11.5, -16), Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(35, -46), Math.toRadians(-90))

                                .waitSeconds(wait)

                                //start first cycle
                                .setReversed(false)

                                .splineToConstantHeading(new Vector2d(11.5, -16), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(11.5, 60), Math.toRadians(90))

                                .setReversed(true)

                                .splineToConstantHeading(new Vector2d(11.5, -16), Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(35, -46), Math.toRadians(-90))

                                .waitSeconds(wait)

                                //start second cycle
                                .setReversed(false)

                                .splineToConstantHeading(new Vector2d(11.5, -16), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(11.5, 60), Math.toRadians(90))

                                .setReversed(true)

                                .splineToConstantHeading(new Vector2d(11.5, -16), Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(35, -46), Math.toRadians(-90))

                                .waitSeconds(wait)

                                //start third cycle
                                .setReversed(false)

                                .splineToConstantHeading(new Vector2d(11.5, -16), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(11.5, 60), Math.toRadians(90))

                                .setReversed(true)

                                .splineToConstantHeading(new Vector2d(11.5, -16), Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(35, -46), Math.toRadians(-90))

                                //start fourth cycle
                                .setReversed(false)

                                .splineToConstantHeading(new Vector2d(11.5, -16), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(11.5, 60), Math.toRadians(90))

                                .setReversed(true)

                                .splineToConstantHeading(new Vector2d(11.5, -16), Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(35, -46), Math.toRadians(-90))

                                .build()
                );

        Image img = null;
        try {img = ImageIO.read(new File("D:/mario/Documents/Robotics/CenterStage.png")); }
        catch (IOException ignored) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(smooth)
//                .addEntity(jank)
                .addEntity(fast)
                .start();
    }
}