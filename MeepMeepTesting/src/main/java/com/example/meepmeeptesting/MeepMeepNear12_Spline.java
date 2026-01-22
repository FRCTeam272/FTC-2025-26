package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepNear12_Spline {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        //TODO - Coordinate List

        // Starting Coordinates
        double startX = -62;
        double startY = -39.5;
        double startH = Math.toRadians(180);

        // Launch Position Preload
        double launchX = -12;
        double launchY = -12;
        double launchH = Math.toRadians(Util.angleToBlueGoalDegrees(launchX, launchY) +180);

        // Launch Position Load1
        double launch1X = -12;
        double launch1Y = -12;
        double launch1H = Math.toRadians(Util.angleToBlueGoalDegrees(launch1X, launch1Y));

        // Launch Position Load2
        double launch2X = -12;
        double launch2Y = -12;
        double launch2H = Math.toRadians(Util.angleToBlueGoalDegrees(launch2X, launch2Y));

        // Launch Position Load3
        double launch3X = -12;
        double launch3Y = -12;
        double launch3H = Math.toRadians(Util.angleToBlueGoalDegrees(launch3X, launch3Y));

        // Go to Pickup Load1 Start
        double load1X = -8;
        double load1Y = -30;
        double load1H = Math.toRadians(270); //Red=90, Blue=270

        // Go to Pickup Load1 End while Intaking
        double getload1X = -8;
        double getload1Y = -59;
        double getload1H = Math.toRadians(270); //Red=90, Blue=270

        // Go to Pickup Load 2 Start
        double load2X = 19;
        double load2Y = -30;
        double load2H = Math.toRadians(270); //Red=90, Blue=270

        // Go to Pickup Load 2 End while Intaking
        double getload2X = 19;
        double getload2Y = -65;
        double getload2H = Math.toRadians(270); //Red=90, Blue=270

        // Backing up from Load 2 to avoid gate
        double avoidGateX = 19;
        double avoidGateY = -48;
        double avoidGateH = Math.toRadians(270); //Red=90, Blue=270

        // Go to Pickup Load 3 Start
        double load3X = 36;
        double load3Y = -30;
        double load3H = Math.toRadians(270); //Red=90, Blue=270

        // Go to Pickup Load 3 End while Intaking
        double getload3X = 36;
        double getload3Y = -65;
        double getload3H = Math.toRadians(270); //Red=90, Blue=270

        // End auto off a launch line, facing away from Driver
        double endX = -52;
        double endY = -20;
        double endH = Math.toRadians(270); //Red=90, Blue = 270

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        //TODO - Write Actions using coordinate names. Use .waitSeconds as a placeholder for robot actions (shooting, intaking, etc) Comment your code!!!!

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(startX, startY, startH)) //starting coordinates
                .waitSeconds(.1)
                        .setReversed(true)
                .splineTo(new Vector2d(launchX,launchY),launchH) //drive to preload shooting position
                .waitSeconds(2.5) //fire preload
                .strafeToLinearHeading(new Vector2d(load1X,load1Y),load1H) //drive to position to loading 1st set of artifacts
                .strafeToLinearHeading(new Vector2d(getload1X,getload1Y),getload1H)
                .strafeToLinearHeading(new Vector2d(launch1X,launch1Y),launch1H) //drive to load1 shooting position
                .waitSeconds(2.5) //fire load1
                .strafeToLinearHeading(new Vector2d(load2X,load2Y),load2H) //drive to position to loading 2nd set of artifacts
                .strafeToLinearHeading(new Vector2d(getload2X,getload2Y),getload2H)
                        .setReversed(true)
                .splineToLinearHeading(new Pose2d(launch2X,launch2Y,launch2H),Math.toRadians(135)) //drive to preload shooting position
                .waitSeconds(2.5) //fire load2
                .strafeToLinearHeading(new Vector2d(load3X,load3Y),load3H) //drive to position to loading 3rd set of artifacts
                .strafeToLinearHeading(new Vector2d(getload3X,getload3Y),getload3H)
                .strafeToLinearHeading(new Vector2d(launch3X,launch3Y),launch3H) //drive to load3 shooting position
                .waitSeconds(2.5) //fire load3
                .strafeToLinearHeading(new Vector2d(endX,endY),endH) //drive to load3 shooting position
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}