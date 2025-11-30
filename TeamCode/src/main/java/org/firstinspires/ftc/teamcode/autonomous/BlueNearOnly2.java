package org.firstinspires.ftc.teamcode.autonomous;

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
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MatchSettings;


@Config
@Autonomous (name="BlueNearOnly2", group="Auto")
public class BlueNearOnly2 extends LinearOpMode {

    private MatchSettings matchSettings;

    private MecanumDrive drive;
    private LauncherSubsystem launcher;
    private IntakeSubsystem intake;

    //TODO - Coordinate List (Pasted from MeepMeep!)

    // Starting Coordinates
    double startX = -63;
    double startY = -36;
    double startH = Math.toRadians(180);

    // Prelaunch Coordinates
    double launchX = -50;
    double launchY = -44;
    double launchH = Math.toRadians(235);

    // Pickup Load1
    double pickup1X = -12;
    double pickup1Y = -24;
    double pickup1H = Math.toRadians(270);

    // Intake Load1
    double intake1X = -12;
    double intake1Y = -46;
    double intake1H = Math.toRadians(270);

    // Launch Load1
    double launch1X = -50;
    double launch1Y = -44;
    double launch1H = Math.toRadians(235);

    // Pickup Load2
    double pickup2X = 12;
    double pickup2Y= -24;
    double pickup2H = Math.toRadians(270);

    // Intake Load2
    double intake2X = 12;
    double intake2Y= -46;
    double intake2H = Math.toRadians(270);

    // Launch Load2
    double launch2X = -50;
    double launch2Y = -44;
    double launch2H = Math.toRadians(235);


    // Ending Coordinates
    double endX = -20;
    double endY = -40;
    double endH = Math.toRadians(235);


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

        LauncherSubsystem launcher = new LauncherSubsystem(hardwareMap, telemetry);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry, matchSettings);

        // TODO Build Trajectories - paste from MeepMeep, separating out by movement,
        // because robot will do other actions timed by where in the trajectory it is
//-------------------------------------------------------------
        //drive to preload shooting position
        TrajectoryActionBuilder goToShootPreload = drive.actionBuilder(StartPose)
                .strafeToLinearHeading(new Vector2d(launchX,launchY),launchH) //drive to preload shooting position
                ;
        Action GoToShootPreload = goToShootPreload.build(); //notice the uppercase name of the Action vs the lower case name of the trajectory!
//-------------------------------------------------------------
        //drive to position to load 1st set of artifacts
        TrajectoryActionBuilder goPickupLoad1 = goToShootPreload.endTrajectory().fresh() // instead of StartPose, it works from where the last trajectory ended
                .strafeToLinearHeading(new Vector2d(pickup1X,pickup1Y),pickup1H) //drive to position to loading 1st set of artifacts
                ;
        Action GoPickupLoad1 = goPickupLoad1.build();

        //-------------------------------------------------------------
        //intake 1st set of artifacts
        TrajectoryActionBuilder intakeLoad1 = goPickupLoad1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(intake1X, intake1Y),intake1H) //drive forward to intake 1st load
                ;
        Action IntakeLoad1 = intakeLoad1.build();

        //-------------------------------------------------------------
        //drive back to launch load 1
        TrajectoryActionBuilder driveToLaunch1 = intakeLoad1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(launch1X, launch1Y),launch1H) //drive to launch point
                ;
        Action DriveToLaunch1 = driveToLaunch1.build();

        //drive to position to load 2nd set of artifacts
        TrajectoryActionBuilder goPickupLoad2 = driveToLaunch1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(pickup2X, pickup2Y),pickup2H) //drive to position to load 2nd set of artifacts
                ;
        Action GoPickupLoad2 = goPickupLoad2.build();

        //intake 2nd set of artifacts
        TrajectoryActionBuilder intakeLoad2 = goPickupLoad2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(intake2X, intake2Y),intake2H) //drive forward to intake load 2
                ;
        Action IntakeLoad2 = intakeLoad2.build();

        //drive back to launch load 2
        TrajectoryActionBuilder driveToLaunch2 = intakeLoad2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(launch2X, launch2Y),launch2H) //drive to launch point
                ;
        Action DriveToLaunch2 = driveToLaunch2.build();



        //drive to ending position
        TrajectoryActionBuilder endingPose = driveToLaunch2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(endX, endY),endH)//end
                ;
        Action EndingPose = endingPose.build();



        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Position during Init", StartPose);
            telemetry.update();
        }

        telemetry.addData("Starting Position", StartPose);
        telemetry.update();

        Actions.runBlocking(new SequentialAction( //overall sequential action that continues for length of Auton

                launcher.autoSetRPMFar(),
                // drive to shoot preload while spinning up motor wheel at the same time
                new ParallelAction(
                        launcher.autoSpinUp(),
                        GoToShootPreload
                ),

                // shoot 3 Artifacts from far position, checking launcher wheel speed between each launch
                launcher.autoCheckAtSpeed(),
                intake.autoLaunch1st(),
                launcher.autoCheckAtSpeed(),
                intake.autoLaunch2nd(),
                launcher.autoCheckAtSpeed(),
                intake.autoLaunch3rd(),

                // drive to intake Load 1
                GoPickupLoad1,

                // Drive forward SLOWLY intaking Artifacts - See trajectory build action for how this is achieved
                new ParallelAction(
                        IntakeLoad1,
                        intake.autoIntake3Front()
                ),

                new SleepAction(1),

                // more stuff here!

                launcher.autoStop() //don't forget to stop launcher at the end!!!!

        ));

        // Stores ending pose for use by Teleop
        matchSettings.setStoredPose(drive.localizer.getPose());

    }
}