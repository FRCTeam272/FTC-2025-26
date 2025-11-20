package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MotifVisionSubsystem;
import org.firstinspires.ftc.teamcode.util.MatchSettings;

@Autonomous (name="Motif Test AutoV2", group = "Tests")
public class MotifAutoV2 extends LinearOpMode {

    private MatchSettings matchSettings;

    @Override
    public void runOpMode() throws InterruptedException {

        matchSettings = new MatchSettings(blackboard);

        // Initialize blackboard with default values to ensure clean state
        // This prevents stale data from previous runs from affecting the current run
        matchSettings.setAllianceColor(MatchSettings.AllianceColor.BLUE);
        matchSettings.setMotif(MatchSettings.Motif.UNKNOWN);

        // Initializing Robot
        Pose2d StartPose = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose);

        MotifVisionSubsystem vision = new MotifVisionSubsystem(hardwareMap, telemetry,matchSettings);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Position during Init", StartPose);
            telemetry.update();
        }

        telemetry.addData("Starting Position", StartPose);
        telemetry.addData("Alliance Color", matchSettings.getAllianceColor());
        telemetry.addData("Motif", matchSettings.getMotif());
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // TODO Build the actual list of actions with Sequential and Parallel Actions
        // Use SleepAction (the number in () is seconds) as placeholders, but many will need to stay because it will move on to the next action
        // as soon as the prior non-movement action is started

        vision.scanObeliskTagSequence();
        Actions.runBlocking(new SequentialAction( //overall sequential action that continues for length of Auton
                new SleepAction(10)
        ));

        telemetry.addData("Starting Position", StartPose);
        telemetry.addData("Alliance Color", matchSettings.getAllianceColor());
        telemetry.addData("Motif", matchSettings.getMotif());
        telemetry.update();
    }
}
