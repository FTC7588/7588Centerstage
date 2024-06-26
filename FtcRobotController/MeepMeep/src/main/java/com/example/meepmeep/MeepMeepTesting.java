package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorManager;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
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

        Pose2d startPose = new Pose2d(60, 36, Math.toRadians(0));

        Pose2d park = new Pose2d(60, -46, Math.toRadians(90));

        RoadRunnerBotEntity one = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 70, Math.toRadians(300), Math.toRadians(280), 14)
                .setColorScheme(new ColorSchemeBlueLight())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                //spike
                                .lineToLinearHeading(new Pose2d(37, 36, Math.toRadians(90)))
                                //side
                                .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(90)))
                                //forward
                                .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(90)))
                                //backdrop
                                .lineToLinearHeading(new Pose2d(44, -52, Math.toRadians(90)))
                                //slow backdrop
                                .lineToLinearHeading(new Pose2d(44, -56, Math.toRadians(90)))
                                //park
                                .lineToLinearHeading(park)
                                .build()
                );

        RoadRunnerBotEntity two = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 70, Math.toRadians(300), Math.toRadians(300), 14)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                //spike
                                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(0)))
                                //side
                                .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(90)))
                                //forward
                                .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(90)))
                                //backdrop
                                .lineToLinearHeading(new Pose2d(37, -52, Math.toRadians(90)))
                                //slow backdrop
                                .lineToLinearHeading(new Pose2d(37, -56, Math.toRadians(90)))
                                //park
                                .lineToLinearHeading(park)
                        .build()
                );

        RoadRunnerBotEntity three = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 70, Math.toRadians(300), Math.toRadians(300), 14)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                //spike
                                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(-90)))
                                //side
                                .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(90)))
                                //forward
                                .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(90)))
                                //backdrop
                                .lineToLinearHeading(new Pose2d(32, -56, Math.toRadians(90)))
                                //slow backdrop
                                .lineToLinearHeading(new Pose2d(32, -56, Math.toRadians(90)))
                                //park
                                .lineToLinearHeading(park)
                                .build()
                );

        Image img = null;
        try {img = ImageIO.read(new File("D:/mario/Documents/Robotics/CenterStage.png")); }
        catch (IOException ignored) {}

        assert img != null;
        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(one)
                .addEntity(two)
                .addEntity(three)
                .start();
    }
}