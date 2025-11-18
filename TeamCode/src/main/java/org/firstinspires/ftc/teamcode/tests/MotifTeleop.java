package org.firstinspires.ftc.teamcode.tests;

import org.firstinspires.ftc.teamcode.util.MatchSettings;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

public class MotifTeleop extends SampleCommandTeleop {
    public MatchSettings matchSettings;


    @Override
    public void onInit() {
        // Pull the stored match state and settings from when they were set during auto
        matchSettings = new MatchSettings(blackboard);

    }

    @Override
    public void onStart() {
        telemetry.addData("Alliance Color", matchSettings.getAllianceColor());
        telemetry.addData("Motif", matchSettings.getMotif());
        telemetry.update();
    }

    @Override
    public void onLoop() {

    }

    @Override
    public void onStop() {

    }
}
