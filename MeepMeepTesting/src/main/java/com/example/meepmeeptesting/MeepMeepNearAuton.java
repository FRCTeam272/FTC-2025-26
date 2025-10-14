package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepNearAuton {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        //TODO - Coordinate List

        // Starting Coordinates
        double startX = -50;
        double startY = -44;
        double startH = Math.toRadians(235);

        // Pickup Load1
        double pickup1X = -12;
        double pickup1Y = -24;
        double pickup1H = Math.toRadians(270);

        // Intake Load1
        double intake1X = -12;
        double intake1Y = -52;
        double intake1H = Math.toRadians(270);

        // Launch Load1
        double launch1X = -50;
        double launch1Y = -44;
        double launch1H = Math.toRadians(235);

        // Pickup Load2
        double pickup2X = 12;
        double pickup2Y= -24;
        double pickup2H = Math.toRadians(270);

        // Intake Load2
        double intake2X = 12;
        double intake2Y= -52;
        double intake2H = Math.toRadians(270);

        // Launch Load2
        double launch2X = -50;
        double launch2Y = -44;
        double launch2H = Math.toRadians(235);

        // Pickup Load3
        double pickup3X = 35.5;
        double pickup3Y= -24;
        double pickup3H = Math.toRadians(270);

        // Intake Load3
        double intake3X = 35.5;
        double intake3Y= -52;
        double intake3H = Math.toRadians(270);

        // Ending Coordinates
        double endX = -20;
        double endY = -40;
        double endH = Math.toRadians(235);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(startX, startY, startH)) //starting coordinates
                .waitSeconds(1)
                .waitSeconds(3) //fire preload
                .strafeToLinearHeading(new Vector2d(pickup1X,pickup1Y),pickup1H) //drive to position to loading 1st set of artifacts
                .strafeToLinearHeading(new Vector2d(intake1X, intake1Y),intake1H) //drive forward to intake load 2
                .strafeToLinearHeading(new Vector2d(launch1X, launch1Y),launch1H) //drive to launch point
                .waitSeconds(3) //fire load 1
                .strafeToLinearHeading(new Vector2d(pickup2X, pickup2Y),pickup2H) //drive to position to load 2nd set of artifacts
                .strafeToLinearHeading(new Vector2d(intake2X, intake2Y),intake2H) //drive forward to intake load 2
                .strafeToLinearHeading(new Vector2d(launch2X, launch2Y),launch2H) //drive to launch point
                .waitSeconds(3) //fire load 2
                .strafeToLinearHeading(new Vector2d(pickup3X,pickup3Y),pickup3H) //drive to position to loading 3rd set of artifacts
                .strafeToLinearHeading(new Vector2d(intake3X, intake3Y),intake3H) //drive forward to intake load 3
                .strafeToLinearHeading(new Vector2d(endX, endY),endH)
                .waitSeconds(1)
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}