package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystemV3;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemV3;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MatchSettings;


@Autonomous (name="BlueNear2", group="Auto")
public class BlueNear2 extends LinearOpMode {

    private MatchSettings matchSettings;

    private MecanumDrive drive;
    private LauncherSubsystemV3 launcher;
    private IntakeSubsystemV3 intake;
    private VisionSubsystem vision;
    private LEDSubsystem leds;

    //TODO - Coordinate List (Pasted from MeepMeep!)

    // Starting Coordinates
    double startX = -62;
    double startY = -39.5;
    double startH = Math.toRadians(180);

    // Launch Position Preload
    double launchX = -12;
    double launchY = -12;
    double launchH = Math.toRadians(Constants.Util.angleToBlueGoalDegrees(launchX, launchY) +180);

    // Launch Position Load1
    double launch1X = -12;
    double launch1Y = -12;
    double launch1H = Math.toRadians(Constants.Util.angleToBlueGoalDegrees(launch1X, launch1Y));

    // Launch Position Load2
    double launch2X = -12;
    double launch2Y = -12;
    double launch2H = Math.toRadians(Constants.Util.angleToBlueGoalDegrees(launch2X, launch2Y));

    // Go to Pickup Load1 Start
    double load1X = -8;
    double load1Y = -30;
    double load1H = Math.toRadians(270); //Red=90, Blue=270

    // Go to Pickup Load1 End while Intaking
    double getload1X = -8;
    double getload1Y = -62;
    double getload1H = Math.toRadians(270); //Red=90, Blue=270

    // Go to Pickup Load 2 Start
    double load2X = 17.5;
    double load2Y = -30;
    double load2H = Math.toRadians(270); //Red=90, Blue=270

    // Go to Pickup Load 2 End while Intaking
    double getload2X = 17.5;
    double getload2Y = -65;
    double getload2H = Math.toRadians(270); //Red=90, Blue=270

    // End auto off a launch line, facing away from Driver
    double endX = -52;
    double endY = -20;
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
                .setReversed(true)
                .splineTo(new Vector2d(launchX,launchY),launchH) //drive to preload shooting position
        ;
        Action GoToLaunchPreload = goToLaunchPreload.build();

        //drive to position to load 1st set of artifacts
        TrajectoryActionBuilder goToIntakeLoad1 = goToLaunchPreload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(load1X,load1Y),load1H) //drive to position to load 1st set of artifacts
                ;
        Action GoToIntakeLoad1 = goToIntakeLoad1.build();

        //get load one, slowly
        TrajectoryActionBuilder  intakeLoad1= goToIntakeLoad1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(getload1X,getload1Y),getload1H, new TranslationalVelConstraint(50.0)) //drive SLOWLY to position to loading 1st set of artifacts
                ;
        Action IntakeLoad1 = intakeLoad1.build();

        //drive back to launch position
        TrajectoryActionBuilder goToLaunchLoad1 = intakeLoad1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(launch1X, launch1Y), launch1H)
                ;
        Action GoToLaunchLoad1 = goToLaunchLoad1.build();

        //drive to position to load 2nd set of artifacts on Wall
        TrajectoryActionBuilder goToIntakeLoad2 = goToLaunchLoad1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(load2X, load2Y), load2H)
                ;
        Action GoToIntakeLoad2 = goToIntakeLoad2.build();

        //get load2, slowly
        TrajectoryActionBuilder  intakeLoad2 = goToIntakeLoad2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(getload2X, getload2Y), getload2H, new TranslationalVelConstraint(50.0)) //drive SLOWLY to position to loading 1st set of artifacts
                ;
        Action IntakeLoad2 = intakeLoad2.build();

        //drive back to launch position
        TrajectoryActionBuilder goToLaunchLoad2 = intakeLoad2.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(launch2X,launch2Y,launch2H),Math.toRadians(135))
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
                new ParallelAction( //leds update during entire auto & vision scans until it saves the motif - run in parallel to everything else
                        leds.updateAuto(),
                        vision.autoScanMotif(),
                        launcher.autoSpinUp(),
                        new SequentialAction(
                                intake.autoResetAutoTimer(), // so that launching can be canceled to get Leave every time
                                launcher.autoSetRPMNear(),
                                // spin to launch position
                                GoToLaunchPreload,

                                // launch 3 Artifacts from far position, checking launcher wheel speed between each launch
                                intake.autoLaunch3Fast(),


                                // stop launcher and drive to Load 1
                                GoToIntakeLoad1,

                                // Drive forward SLOWLY intaking Artifacts
                                new ParallelAction(
                                        IntakeLoad1,
                                        intake.autoIntake3Front(),
                                        intake.autoCloseColors()
                                ),

                                // spin up launcher and drive to launch position for Load 1
                                GoToLaunchLoad1,

                                // launch 3 Artifacts from far position
                                intake.autoLaunch3Fast(),

                                //stop Launcher and drive to Load 2 at the wall
                                GoToIntakeLoad2,

                                // Drive forward SLOWLY intaking Artifacts from the wall.
                                new ParallelAction(
                                        IntakeLoad2,
                                        intake.autoIntake3Front(),
                                        intake.autoMidColors()
                                ),
                                GoToLaunchLoad2,

                                // launch 3 Artifacts from far position, checking launcher wheel speed between each launch
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
