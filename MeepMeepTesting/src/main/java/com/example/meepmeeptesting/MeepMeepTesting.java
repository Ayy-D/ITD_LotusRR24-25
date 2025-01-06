package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 60, Math.toRadians(270)))
                        //Preload Specimen Score
                        .lineToY(34.5)

                        //Move to grab floor spec 1
                        .strafeToLinearHeading(new Vector2d(-34, 42), Math.toRadians(45))

                        //Move to drop floor spec 1
                        .strafeToLinearHeading(new Vector2d(-36, 48), Math.toRadians(315))

                        //Move to grab floor spec 2
                        .strafeToLinearHeading(new Vector2d(-44, 42), Math.toRadians(45))

                        //Move to drop floor spec 2
                        .strafeToLinearHeading(new Vector2d(-48, 48), Math.toRadians(315))

                        //Move to grab floor spec 3
                        .strafeToLinearHeading(new Vector2d(-54, 42), Math.toRadians(45))

                        //Move to drop floor spec 3
                        .strafeToLinearHeading(new Vector2d(-54, 48), Math.toRadians(315))

                        //Move to grab spec 1 from wall
                        .strafeToLinearHeading(new Vector2d(-40, 54), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(-40, 58), Math.toRadians(90))

                        //Move to score spec 1
                        .strafeToSplineHeading(new Vector2d(0, 34), Math.toRadians(270))

                        //Move to grab spec 2 from wall
                        .strafeToLinearHeading(new Vector2d(-40, 54), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(-40, 58), Math.toRadians(90))

                        //Move to score spec 2
                        .strafeToSplineHeading(new Vector2d(0, 34), Math.toRadians(270))

                        //Move to grab spec 3 from wall
                        .strafeToLinearHeading(new Vector2d(-40, 54), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(-40, 58), Math.toRadians(90))

                        //Move to score spec 3
                        .strafeToSplineHeading(new Vector2d(0, 34), Math.toRadians(270))

                        //Park
                        .strafeToLinearHeading(new Vector2d(-24, 45), Math.toRadians(315))



                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}