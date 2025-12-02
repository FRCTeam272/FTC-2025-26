package org.firstinspires.ftc.teamcode.autonomous.archive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystemV2;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemV2;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MatchSettings;


@Config
@Autonomous (name="RedFarTestCleanIntakeOnly", group="Tests")
public class RedFarTestCleanIntakeOnly extends LinearOpMode {

    private MatchSettings matchSettings;

    private MecanumDrive drive;
    private LauncherSubsystemV2 launcher;
    private IntakeSubsystemV2 intake;
    private LEDSubsystem leds;

    //TODO - Coordinate List (Pasted from MeepMeep!)

    // Starting Coordinates
    double startX = 61;
    double startY = 15;
    double startH = Math.toRadians(180);

    // Shoot Preload
    double preloadX = 58;
    double preloadY = 15;
    double preloadH = Constants.Util.angleToRedGoal(preloadX,preloadY);

    // Go to Pickup Load1 Start
    double load1X = 37;
    double load1Y = 30;
    double load1H = Math.toRadians(90);

    // Go to Pickup Load1 End while Intaking
    double getload1X = 37;
    double getload1Y = 52;
    double getload1H = Math.toRadians(90);

    @Override
    public void runOpMode() throws InterruptedException {

        matchSettings = new MatchSettings(blackboard);
        blackboard.clear(); //do not save match settings between matches

        // Initialize blackboard with default values to ensure clean state
        // This prevents stale data from previous runs from affecting the current run
        matchSettings.setAllianceColor(MatchSettings.AllianceColor.RED);

        // Initializing Robot
        Pose2d StartPose = new Pose2d(startX,startY,startH);
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose);
        drive.localizer.setPose(StartPose);

        LauncherSubsystemV2 launcher = new LauncherSubsystemV2(hardwareMap, telemetry, matchSettings);
        IntakeSubsystemV2 intake = new IntakeSubsystemV2(hardwareMap, telemetry, matchSettings);
        LEDSubsystem leds = new LEDSubsystem(hardwareMap,matchSettings);

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

        //get load one
        TrajectoryActionBuilder  intakeload1= goToIntakeLoad1.endTrajectory().fresh() // instead of StartPose, it works from where the last trajectory ended
                .strafeToLinearHeading(new Vector2d(getload1X,getload1Y),getload1H, new TranslationalVelConstraint(20.0)) //drive SLOWLY to position to loading 1st set of artifacts
                ;
        Action Intakeload1 = intakeload1.build();


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
                new ParallelAction( //leds update during entire auto - run in parallel to everything else
                        leds.updateAuto(),
                        new SequentialAction(
                                launcher.autoSetRPMFar(),
                                // drive to shoot preload while spinning up motor wheel at the same time
                                new ParallelAction(
//                                        launcher.autoSpinUp(),
                                        GoToShootPreload
                                ),

                                // shoot 3 Artifacts from far position, checking launcher wheel speed between each launch
//                                launcher.autoCheckAtSpeed(),
//                                intake.autoLaunch1st(),
//                                launcher.autoCheckAtSpeed(),
//                                intake.autoLaunch2nd(),
//                                launcher.autoCheckAtSpeed(),
//                                intake.autoLaunch3rd(),

                                // drive to intake Load 1
                                GoToIntakeLoad1,

                                // Drive forward SLOWLY intaking Artifacts - See trajectory build action for how this is achieved
                                new ParallelAction(
                                        Intakeload1,
                                        intake.autoIntake3Front()
                                ),

                                new SleepAction(1)
                                //,

                                // more stuff here!

                                //launcher.autoStop() //don't forget to stop launcher at the end!!!!
                        )
                )));

        // Stores ending pose for use by Teleop
        matchSettings.setStoredPose(drive.localizer.getPose());

        while (!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

    }
}
