package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBlueFar12_Spline {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        //TODO - Coordinate List

        // Starting Coordinates
        double startX = 62;
        double startY = -15;
        double startH = Math.toRadians(180);

        // Launch Preload
        double launchX = 55;
        double launchY = -15;
        double launchH = Math.toRadians(Util.angleToBlueGoalDegrees(launchX, launchY)-5); // -5 blue, +3 red

        // Launch Load1
        double launch1X = 55;
        double launch1Y = -15;
        double launch1H = Math.toRadians(Util.angleToBlueGoalDegrees(launch1X, launch1Y) +180);

        // Launch Load2
        double launch2X = 55;
        double launch2Y = -15;
        double launch2H = Math.toRadians(Util.angleToBlueGoalDegrees(launch2X, launch2Y) +180);

        // Launch Load3
        double launch3X = 55;
        double launch3Y = -15;
        double launch3H = Math.toRadians(Util.angleToBlueGoalDegrees(launch3X, launch3Y) +180);

        // Go to Pickup Load1 Start
        double load1X = 37;
        double load1Y = -30;
        double load1H = Math.toRadians(270); //Red=90, Blue=270

        // Go to Pickup Load1 End while Intaking
        double getload1X = 37;
        double getload1Y = -65;
        double getload1H = Math.toRadians(270); //Red=90, Blue=270

        // Go to Pickup Load 2 Start
        double load2X = 16;
        double load2Y = -30;
        double load2H = Math.toRadians(270); //Red=90, Blue=270

        // Go to Pickup Load 2 End while Intaking
        double getload2X = 16;
        double getload2Y = -65;
        double getload2H = Math.toRadians(270); //Red=90, Blue=270

        // Go to Pickup Wall Load Start
        double load3wallX = 48;
        double load3wallY = -59;
        double load3wallH = Math.toRadians(310); //Red=50, Blue=310

        // Go to Pickup Wall Load End while Intaking
        double getload3wallX = 68;
        double getload3wallY = -64.5;
        double getload3wallH = Math.toRadians(0); //Red=50, Blue=310

        // End auto off a launch line, facing away from Driver
        double endX = 36;
        double endY = -24;
        double endH = Math.toRadians(270); //Red=90, Blue = 270

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        //TODO - Write Actions using coordinate names. Use .waitSeconds as a placeholder for robot actions (shooting, intaking, etc) Comment your code!!!!

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(startX, startY, startH)) //starting coordinates
                .waitSeconds(1)
                .splineTo(new Vector2d(launchX,launchY),launchH) //drive to preload shooting position
                .waitSeconds(2) //fire preload
                .splineTo(new Vector2d(load1X, load1Y), load1H) //drive to position to loading 1st set of artifacts
                .splineTo(new Vector2d(getload1X, getload1Y), getload1H) // intake load1
                        .setReversed(true)
                .splineTo(new Vector2d(launch1X,launch1Y),launch1H)
                        .waitSeconds(2)//fire load1
                        .setReversed(false)
                        .splineTo(new Vector2d(load2X,load2Y),load2H)
                .splineTo(new Vector2d(getload2X, getload2Y), getload2H) // intake load2
                .setReversed(true)
                .splineTo(new Vector2d(launch2X,launch2Y),launch2H)
                .waitSeconds(2)//fire load2
                        .setReversed(false)
                .splineTo(new Vector2d(load3wallX, load3wallY), load3wallH) // intake load2
                        .splineTo(new Vector2d(getload3wallX,getload3wallY),getload3wallH)
                .setReversed(true)
                .splineTo(new Vector2d(launch3X,launch3Y),launch3H)
                .waitSeconds(2)//fire load3
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}