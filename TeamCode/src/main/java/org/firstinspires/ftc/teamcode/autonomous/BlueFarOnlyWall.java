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
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystemV3;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemV3;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MatchSettings;


@Autonomous (name="BlueFarOnlyWall", group="Auto")
public class BlueFarOnlyWall extends LinearOpMode {

    private MatchSettings matchSettings;

    private MecanumDrive drive;
    private LauncherSubsystemV3 launcher;
    private IntakeSubsystemV3 intake;
    private VisionSubsystem vision;
    private LEDSubsystem leds;

    //TODO - Coordinate List (Pasted from MeepMeep!)

    // Starting Coordinates
    double startX = 62;
    double startY = -15;
    double startH = Math.toRadians(180);

    // Launch Preload
    double launchX = 55;
    double launchY = -15;
    double launchH = Math.toRadians(Constants.Util.angleToBlueGoalDegrees(launchX, launchY));

    // Launch Load1
    double launch1X = 55;
    double launch1Y = -15;
    double launch1H = Math.toRadians(Constants.Util.angleToBlueGoalDegrees(launch1X, launch1Y));

    // Go to Pickup Wall Load Start
    double load1wallX = 56;
    double load1wallY = -64.5;
    double load1wallH = Math.toRadians(310); //Red=50, Blue=310

    // Go to Pickup Wall Load End while Intaking
    double getload1wallX = 68;
    double getload1wallY = -64.5;
    double getload1wallH = Math.toRadians(310); //Red=50, Blue=310

    // End auto off a launch line, facing away from Driver
    double endX = 55;
    double endY = -36;
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
        IntakeSubsystemV3 intake = new IntakeSubsystemV3(hardwareMap, telemetry, matchSettings);
        VisionSubsystem vision = new VisionSubsystem(hardwareMap,matchSettings);
        LEDSubsystem leds = new LEDSubsystem(hardwareMap,matchSettings);

        // TODO Build Trajectories - paste from MeepMeep, separating out by movement,
        // because robot will do other actions timed by where in the trajectory it is

        //drive to preload launch position
        TrajectoryActionBuilder goToLaunchPreload = drive.actionBuilder(StartPose)
                .splineTo(new Vector2d(launchX, launchY), launchH)
                ;
        Action GoToLaunchPreload = goToLaunchPreload.build();

        //drive to position to load 2nd set of artifacts on Wall
        TrajectoryActionBuilder goToIntakeLoad1 = goToLaunchPreload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(load1wallX, load1wallY), load1wallH)
                ;
        Action GoToIntakeLoad1 = goToIntakeLoad1.build();

        //get wall load, slowly
        TrajectoryActionBuilder  intakeLoad1 = goToIntakeLoad1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(getload1wallX, getload1wallY), getload1wallH, new TranslationalVelConstraint(30.0)) //drive SLOWLY to position to loading 1st set of artifacts
                ;
        Action IntakeLoad1 = intakeLoad1.build();

        //drive back to launch position
        TrajectoryActionBuilder goToLaunchLoad1 = intakeLoad1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(launch1X, launch1Y), launch1H)
                ;
        Action GoToLaunchLoad1 = goToLaunchLoad1.build();

        //end Auto off a launch line, facing away from driver
        TrajectoryActionBuilder endAuto = goToLaunchLoad1.endTrajectory().fresh()
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

                                // drive to launch position
                                GoToLaunchPreload,
                                new SleepAction(0.4),

                                // launch Preload - 3 Artifacts from far position
                                intake.autoLaunch3Fast(),

                                // stop launcher and drive to Load 1
                                GoToIntakeLoad1,

                                // Drive forward SLOWLY intaking Artifacts from wall
                                new ParallelAction(
                                        IntakeLoad1,
                                        intake.autoIntake3Front(),
                                        intake.autoFarColors()
                                ),

                                // spin up launcher and drive to launch position for Load 1
                               GoToLaunchLoad1,

                                // launch 3 Artifacts from far position
                                intake.autoLaunch3Fast(),

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
