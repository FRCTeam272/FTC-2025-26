package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.MatchSettings;


@Config
@Autonomous (name="AutonTest", group="Auto")
public class AutonTest extends LinearOpMode {

    private MatchSettings matchSettings;

    //TODO - Coordinate List (Pasted from MeepMeep!)

    // Starting Coordinates
    double startX = 0;
    double startY = 0;
    double startH = Math.toRadians(180);

    // Shoot Preload
    double preloadX = 12;
    double preloadY = 12;
    double preloadH = Math.toRadians(180);

    // Pickup Load1
    double load1X = 24;
    double load1Y = 24;
    double load1H = Math.toRadians(180);

    @Override
    public void runOpMode() throws InterruptedException {

        matchSettings = new MatchSettings(blackboard);

        // Initialize blackboard with default values to ensure clean state
        // This prevents stale data from previous runs from affecting the current run
        matchSettings.setAllianceColor(MatchSettings.AllianceColor.BLUE);

        // Initializing Robot
        Pose2d StartPose = new Pose2d(startX,startY,startH);
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose);

        // TODO Build Trajectories - paste from MeepMeep, separating out by movement,
        // because robot will do other actions timed by where in the trajectory it is

        //drive to preload shooting position
        TrajectoryActionBuilder goToShootPreload = drive.actionBuilder(StartPose)
                .strafeToLinearHeading(new Vector2d(preloadX,preloadY),preloadH) //drive to preload shooting position
                ;
        Action GoToShootPreload = goToShootPreload.build(); //notice the uppercase name of the Action vs the lower case name of the trajectory!

        //drive to position to loading 1st set of artifacts
        TrajectoryActionBuilder goToIntakeLoad1 = goToShootPreload.endTrajectory().fresh() // instead of StartPose, it works from where the last trajectory ended
                .strafeToLinearHeading(new Vector2d(load1X,load1Y),load1H) //drive to position to loading 1st set of artifacts
                ;
        Action GoToIntakeLoad1 = goToIntakeLoad1.build();


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Position during Init", StartPose);
            telemetry.update();
        }

        telemetry.addData("Starting Position", StartPose);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // TODO Build the actual list of actions with Sequential and Parallel Actions
        // Use SleepAction (the number in () is seconds) as placeholders, but many will need to stay because it will move on to the next action
        // as soon as the prior non-movement action is started


        Actions.runBlocking(new SequentialAction( //overall sequential action that continues for length of Auton

                GoToShootPreload,

                GoToIntakeLoad1






        ));

    }
}
