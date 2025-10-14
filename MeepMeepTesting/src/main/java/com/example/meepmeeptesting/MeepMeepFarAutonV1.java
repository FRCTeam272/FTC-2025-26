package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepFarAutonV1 {
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


        // Drive to Pickup Load1
        double getload1X = 35;
        double getload1Y = -52;
        double getload1H = Math.toRadians(270);


        // turn to drive Load1
        double turnload1X = 35;
        double turnload1Y = -28;
        double turnload1H = Math.toRadians(180);


        // drive to shoot Load1
        double driveload1X =-25;
        double driveload1Y = -28;
        double driveload1H = Math.toRadians(180);


        // turn to shoot Load1
        double turntoshootload1X =-26;
        double turntoshootload1Y = -28;
        double turntoshootload1H = Math.toRadians(225);


        //get into position to get load 2
        double load2X =-12;
        double load2Y = -28;
        double load2H = Math.toRadians(270);


        //get load 2
        double getload2X =-12;
        double getload2Y = -52;
        double getload2H = Math.toRadians(270);


        //shoot load 2
        double shootload2X =-26;
        double shootload2Y = -28;
        double shootload2H = Math.toRadians(225);


        //get into position to get load 3
        double load3X =12;
        double load3Y = -28;
        double load3H = Math.toRadians(270);


        //get load 3
        double getload3X =12;
        double getload3Y = -52;
        double getload3H = Math.toRadians(270);


        //turn turn drive load 3 to shooting postion
        double turnload3X =12;
        double turnload3Y = -28;
        double turnload3H = Math.toRadians(180);


        //shoot load 3
        double shootload3X =-26;
        double shootload3Y = -28;
        double shootload3H = Math.toRadians(225);


        //go to end position
        double endX =0;
        double endY = -28;
        double endH = Math.toRadians(270);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(startX, startY, startH)) //starting coordinates
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(preloadX,preloadY),preloadH) //drive to preload shooting position
                .waitSeconds(2) //fire preload
                .strafeToLinearHeading(new Vector2d(load1X,load1Y),load1H) //drive to position to loading 1st set of artifacts
                .strafeToLinearHeading(new Vector2d(getload1X,getload1Y),getload1H) //pickup load 1
                .strafeToLinearHeading(new Vector2d(turnload1X,turnload1Y),turnload1H) //turn to drive load 1 to shooting position
                .strafeToLinearHeading(new Vector2d(driveload1X,driveload1Y),driveload1H)//drive to shooting position for load 1
                .strafeToLinearHeading(new Vector2d(turntoshootload1X,turntoshootload1Y),turntoshootload1H)//turn to shoot load 1
                .waitSeconds(2) //fire load 1
                .strafeToLinearHeading(new Vector2d(load2X,load2Y),load2H)//get into position to get load 2
                .strafeToLinearHeading(new Vector2d(getload2X,getload2Y),getload2H)//get load 2
                .strafeToLinearHeading(new Vector2d(shootload2X,shootload2Y),shootload2H)//shoot load 2
                .waitSeconds(2) //fire load 2
                .strafeToLinearHeading(new Vector2d(load3X,load3Y),load3H)//get into position to get load 3
                .strafeToLinearHeading(new Vector2d(getload3X,getload3Y),getload3H)//get load 3
                .strafeToLinearHeading(new Vector2d(turnload3X,turnload3Y),turnload3H)//turn to drive load 3 to shooting position
                .strafeToLinearHeading(new Vector2d(shootload3X,shootload3Y),shootload3H)//shoot load 3
                .waitSeconds(2) //fire load 3
                .strafeToLinearHeading(new Vector2d(endX,endY),endH)//go to end position
                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

