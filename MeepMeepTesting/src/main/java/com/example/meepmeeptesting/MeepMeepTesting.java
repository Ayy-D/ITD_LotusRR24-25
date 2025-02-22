package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width (in)
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15.5)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(37.25, 61, Math.toRadians(90)))

                //Preload Specimen Score
                .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(45))

                //pick 1
                .strafeToLinearHeading(new Vector2d(52, 52), Math.toRadians(82.5))
                .strafeToLinearHeading(new Vector2d(51, 48), Math.toRadians(82.5))
                .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(45))

                //pick 2
                .strafeToLinearHeading(new Vector2d(56, 52), Math.toRadians(92))
                .lineToY(48)
                .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(45))

                //pick 3
                .strafeToLinearHeading(new Vector2d(57, 50), Math.toRadians(115))
                .lineToY(48)
                .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(45))

                //sub pick 1
                .strafeToLinearHeading(new Vector2d(40, 12), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(25, 12), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(40, 12), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(45))

                //sub pick 2
                .strafeToLinearHeading(new Vector2d(40, 8), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(25, 8), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(40, 8), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(45))

                //Park
                .strafeToLinearHeading(new Vector2d(55, 16), Math.toRadians(150))
                .strafeToLinearHeading(new Vector2d(32, 12), Math.toRadians(180))

                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}