package org.firstinspires.ftc.teamcode.autonomous;

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
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemV3;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MatchSettings;


@Autonomous (name="BlueFarWallDriveAlong", group="Auto")
public class BlueFarWallDriveAlong extends LinearOpMode {

    private MatchSettings matchSettings;

    private MecanumDrive drive;
    private LauncherSubsystemV3 launcher;
    private IntakeSubsystemV2 intake;
    private VisionSubsystem vision;
    private LEDSubsystem leds;

    //TODO - Coordinate List (Pasted from MeepMeep!)

    // Starting Coordinates
    double startX = 62;
    double startY = -15;
    double startH = Math.toRadians(180);

    // Look at Motif
    double motifX = 36;
    double motifY = -15;
    double motifH = Math.toRadians(Constants.Util.angleToMotifDegrees(motifX,motifY));

    // Launch Preload
    double launchX = 55;
    double launchY = -15;
    double launchH = Math.toRadians(Constants.Util.angleToBlueGoalDegrees(launchX, launchY)-5);

    // Launch Load1
    double launch1X = 55;
    double launch1Y = -15;
    double launch1H = Math.toRadians(Constants.Util.angleToBlueGoalDegrees(launch1X, launch1Y));

    // Launch Load2
    double launch2X = 55;
    double launch2Y = -22;
    double launch2H = Math.toRadians(Constants.Util.angleToBlueGoalDegrees(launch2X, launch2Y));

    // Go to Pickup Load1 Start
    double load1X = 37;
    double load1Y = -30;
    double load1H = Math.toRadians(270); //Red=90, Blue=270

    // Go to Pickup Load1 End while Intaking
    double getload1X = 37;
    double getload1Y = -65;
    double getload1H = Math.toRadians(270); //Red=90, Blue=270

    // Go to Pickup Wall Load Start
    double load2wallX = 40;
    double load2wallY = -62;
    double load2wallH = Math.toRadians(0);

    // Go to Pickup Wall Load End while Intaking
    double getload2wallX = 60;
    double getload2wallY = -62;
    double getload2wallH = Math.toRadians(0);

    // End auto off a launch line, facing away from Driver
    double endX = 36;
    double endY = -24;
    double endH = Math.toRadians(270); //Red=90, Blue = 270

    @Override
    public void runOpMode() throws InterruptedException {

        matchSettings = new MatchSettings(blackboard);
        blackboard.clear(); //do not save match settings between matches

        // Initialize blackboard with default values to ensure clean state
        // This prevents stale data from previous runs from affecting the current run
        matchSettings.setAllianceColor(MatchSettings.AllianceColor.BLUE);

        // Initializing Robot
        Pose2d StartPose = new Pose2d(startX,startY,startH);
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose);
        drive.localizer.setPose(StartPose);

        LauncherSubsystemV3 launcher = new LauncherSubsystemV3(hardwareMap, telemetry, matchSettings);
        IntakeSubsystemV2 intake = new IntakeSubsystemV2(hardwareMap, telemetry, matchSettings);
        VisionSubsystem vision = new VisionSubsystem(hardwareMap,matchSettings);
        LEDSubsystem leds = new LEDSubsystem(hardwareMap,matchSettings);

        // TODO Build Trajectories - paste from MeepMeep, separating out by movement,
        // because robot will do other actions timed by where in the trajectory it is

        //drive to motif view position
        TrajectoryActionBuilder goToMotif = drive.actionBuilder(StartPose)
                .strafeToLinearHeading(new Vector2d(motifX, motifY), motifH)
                ;
        Action GoToMotif = goToMotif.build();

        //drive to preload launch position
        TrajectoryActionBuilder goToLaunchPreload = goToMotif.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(launchX, launchY), launchH)
                ;
        Action GoToLaunchPreload = goToLaunchPreload.build();

        //drive to position to load 1st set of artifacts
        TrajectoryActionBuilder goToIntakeLoad1 = goToLaunchPreload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(load1X,load1Y),load1H) //drive to position to load 1st set of artifacts
                ;
        Action GoToIntakeLoad1 = goToIntakeLoad1.build();

        //get load one, slowly
        TrajectoryActionBuilder  intakeLoad1= goToIntakeLoad1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(getload1X,getload1Y),getload1H, new TranslationalVelConstraint(20.0)) //drive SLOWLY to position to loading 1st set of artifacts
                ;
        Action IntakeLoad1 = intakeLoad1.build();

        //drive back to launch position
        TrajectoryActionBuilder goToLaunchLoad1 = intakeLoad1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(launch1X, launch1Y), launch1H)
                ;
        Action GoToLaunchLoad1 = goToLaunchLoad1.build();

        //drive to position to load 2nd set of artifacts on Wall
        TrajectoryActionBuilder goToIntakeLoad2 = goToLaunchLoad1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(load2wallX,load2wallY),load2wallH)
                ;
        Action GoToIntakeLoad2 = goToIntakeLoad2.build();

        //get wall load, slowly
        TrajectoryActionBuilder  intakeLoad2 = goToIntakeLoad2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(getload2wallX,getload2wallY),getload2wallH, new TranslationalVelConstraint(20.0)) //drive SLOWLY to position to loading 1st set of artifacts
                ;
        Action IntakeLoad2 = intakeLoad2.build();

        //drive back to launch position
        TrajectoryActionBuilder goToLaunchLoad2 = intakeLoad2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(launch2X, launch2Y), launch2H)
                ;
        Action GoToLaunchLoad2 = goToLaunchLoad2.build();

        //end Auto off a launch line, facing away from driver
        TrajectoryActionBuilder endAuto = goToLaunchLoad2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(endX, endY), endH)
                ;
        Action EndAuto = endAuto.build();


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
                        vision.autoScanMotif(),
                        launcher.autoSpinUp(),
                        new SequentialAction(
                                intake.autoResetAutoTimer(), // so that launching can be canceled to get Leave every time
                                launcher.autoSetRPMFar(),

                                //go to motif scan position and be still for 1 second while spinning up wheel
                                GoToMotif,
                                new SleepAction(0.4),

                                // drive to launch position
                                GoToLaunchPreload,

                                // launch Preload - 3 Artifacts from far position
                                intake.autoLaunch1st(),
                                intake.autoLaunch2nd(),
                                intake.autoLaunch3rd(),
                                //intake.autoSpitOut(),

                                // stop launcher and drive to Load 1
                                GoToIntakeLoad1,

                                // Drive forward SLOWLY intaking Artifacts
                                new ParallelAction(
                                        IntakeLoad1,
                                        intake.autoIntake3Front(),
                                        intake.autoFarColors()
                                ),

                                // spin up launcher and drive to launch position for Load 1
                               GoToLaunchLoad1,

                                // launch 3 Artifacts from far position
                                intake.autoLaunch1st(),
                                intake.autoLaunch2nd(),
                                intake.autoLaunch3rd(),
                                //intake.autoSpitOut(),

                                //stop Launcher and drive to Load 2 at the wall
                                GoToIntakeLoad2,

                                // Drive forward SLOWLY intaking Artifacts from the wall.
                                new ParallelAction(
                                        IntakeLoad2,
                                        intake.autoIntake3Front(),
                                        intake.autoWallColors()
                                ),
                                GoToLaunchLoad2,

                                // launch 3 Artifacts from far position, checking launcher wheel speed between each launch
                                intake.autoLaunch1st(),
                                intake.autoLaunch2nd(),
                                intake.autoLaunch3rd(),
                                //intake.autoSpitOut(),

                                //stop launcher and drive to end position off launch lines
                                launcher.autoStop(),
                                EndAuto
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
