package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingCleanExample {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        //TODO - Coordinate List

        // Starting Coordinates
        double startX = 62;
        double startY = -15;
        double startH = Math.toRadians(180);

        // Shoot Preload
        double preloadX = 58;
        double preloadY = -15;
        double preloadH = Math.toRadians(210);

        // Pickup Load1
        double load1X = 35;
        double load1Y = -30;
        double load1H = Math.toRadians(270);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(startX, startY, startH)) //starting coordinates
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(preloadX,preloadY),preloadH) //drive to preload shooting position
                .waitSeconds(2) //fire preload
                .strafeToLinearHeading(new Vector2d(load1X,load1Y),load1H) //drive to position to loading 1st set of artifacts
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}