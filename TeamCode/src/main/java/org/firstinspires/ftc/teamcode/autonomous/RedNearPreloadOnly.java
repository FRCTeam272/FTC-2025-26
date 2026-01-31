package org.firstinspires.ftc.teamcode.autonomous;

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
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystemV3;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemV3;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MatchSettings;


@Autonomous (name="RedNearPreloadOnly", group="Auto")
public class RedNearPreloadOnly extends LinearOpMode {

    private MatchSettings matchSettings;

    private MecanumDrive drive;
    private LauncherSubsystemV3 launcher;
    private IntakeSubsystemV3 intake;
    private VisionSubsystem vision;
    private LEDSubsystem leds;

    //TODO - Coordinate List (Pasted from MeepMeep!)

    // Starting Coordinates
    double startX = -62;
    double startY = 39.5;
    double startH = Math.toRadians(180);

    // Launch Position Preload
    double launchX = -48;
    double launchY = 12;
    double launchH = Math.toRadians(Constants.Util.angleToRedGoalDegrees(launchX, launchY));

    // End auto off a launch line, facing away from Driver
    double endX = -54;
    double endY = 20;
    double endH = Math.toRadians(90); //Red=90, Blue = 270

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

        LauncherSubsystemV3 launcher = new LauncherSubsystemV3(hardwareMap, telemetry, matchSettings);
        IntakeSubsystemV3 intake = new IntakeSubsystemV3(hardwareMap, telemetry, matchSettings);
        VisionSubsystem vision = new VisionSubsystem(hardwareMap,matchSettings);
        LEDSubsystem leds = new LEDSubsystem(hardwareMap,matchSettings);

        // TODO Build Trajectories - paste from MeepMeep, separating out by movement,
        // because robot will do other actions timed by where in the trajectory it is

        TrajectoryActionBuilder goToLaunchPreload = drive.actionBuilder(StartPose)
                .strafeToLinearHeading(new Vector2d(launchX, launchY), launchH)
                ;
        Action GoToLaunchPreload = goToLaunchPreload.build();

        //end Auto off a launch line, facing away from driver
        TrajectoryActionBuilder endAuto = goToLaunchPreload.endTrajectory().fresh()
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
                                new SleepAction(0.5),
                                // spin to launch position
                                GoToLaunchPreload,

                                // launch 3 Artifacts from near position, checking launcher wheel speed between each launch
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
