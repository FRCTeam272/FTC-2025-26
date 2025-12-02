package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MatchSettings;


@Config
@Autonomous (name="RedFarTestCleanIntakeOnly", group="Tests")
public class RedFarTestCleanIntakeOnly extends BlueFarTestCleanIntakeOnly {

    private MatchSettings matchSettings;

    @Override
    public void runOpMode() throws InterruptedException {

        //TODO - Coordinate List (Pasted from MeepMeep!)
        //For Red conversion, ALL Y coordinates to POSITIVE. All 270 to 90. (180 and 0 stay the same), change Aiming angle calc to Red goal

        // Starting Coordinates
        double startX = 62;
        double startY = 15;
        double startH = Math.toRadians(180);

        // Shoot Preload
        double preloadX = 58;
        double preloadY = 15;
        double preloadH = Constants.Util.angleToRedGoal(preloadX,preloadY); //calculates angle to goal, no guessing!

        // Pickup Load1
        double load1X = 37;
        double load1Y = 30;
        double load1H = Math.toRadians(90);

        // Drive to Pickup Load1
        double getload1X = 37;
        double getload1Y = 52;
        double getload1H = Math.toRadians(90);

        matchSettings = new MatchSettings(blackboard);
        blackboard.clear(); //do not save match settings between matches

        // Initialize blackboard with default values to ensure clean state
        // This prevents stale data from previous runs from affecting the current run
        matchSettings.setAllianceColor(MatchSettings.AllianceColor.RED);

    }
}
