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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 61, Math.toRadians(270)))
                //Preload Specimen Score
                .lineToY(34.5)

                //Prep to grab floor spec 1
                .strafeToSplineHeading(new Vector2d(-30, 36), Math.toRadians(50))


                //Move to grab floor spec 1
                .strafeToLinearHeading(new Vector2d(-30, 56), Math.toRadians(-15))


                //Move to drop floor spec 1
                .strafeToLinearHeading(new Vector2d(-38, 36), Math.toRadians(50))

                //Prep to grab floor spec 2
                .strafeToLinearHeading(new Vector2d(-38, 56), Math.toRadians(-15))


                //Move to sweep floor spec 2
                .strafeToLinearHeading(new Vector2d(-40, 22), Math.toRadians(15))

                //Move to drop floor spec 2
                .strafeToLinearHeading(new Vector2d(-40, 58), Math.toRadians(0))


                //Move to grab spec 1 from wall
                .strafeToLinearHeading(new Vector2d(-40, 54), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-40, 58), Math.toRadians(90))

                //Move to score spec 1
                .strafeToSplineHeading(new Vector2d(0, 34), Math.toRadians(-90))

                //Move to grab spec 2 from wall
                .strafeToLinearHeading(new Vector2d(-40, 54), Math.toRadians(89.9999))
                .strafeToLinearHeading(new Vector2d(-40, 58), Math.toRadians(90))

                //Move to score spec 2
                .strafeToSplineHeading(new Vector2d(0, 34), Math.toRadians(-90))

                //Move to grab spec 3 from wall
                .strafeToLinearHeading(new Vector2d(-40, 54), Math.toRadians(89.9999))
                .strafeToLinearHeading(new Vector2d(-40, 58), Math.toRadians(90))

                //Move to score spec 3
                .strafeToSplineHeading(new Vector2d(0, 34), Math.toRadians(-90))



                //Park
                .strafeToLinearHeading(new Vector2d(-40, 58), Math.toRadians(315))

                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}