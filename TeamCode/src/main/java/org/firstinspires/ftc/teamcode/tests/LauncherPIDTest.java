package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemV3;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MatchSettings;

@Disabled
@TeleOp (name = "LauncherPIDTest", group = "Tests")
public class LauncherPIDTest extends CommandOpMode {

    LauncherSubsystemV3 launcher;
    private FtcDashboard dash;
    TelemetryPacket packet = new TelemetryPacket();
    MatchSettings matchSettings;

    double[] stepSizes = {10, 1, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    @Override
    public void initialize() {

        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        launcher = new LauncherSubsystemV3(hardwareMap, telemetry, matchSettings);

        telemetry.addLine("init done");
        telemetry.update();

        launcher.setTargetRPM(0);
        launcher.setF(0);
        launcher.setP(0);
    }

    // --- PIDF ---
    /**
     * Sets launcher PIDF coefficients manually in LauncherSubsystem Configs
     * Dashboard address - 192.168.43.1:8080/dash
     * Tune F first! Increase F until it reaches target velocity. BP set his at 14.098. Consider setting for each target velocity
     * Next Tune P. Want it to quickly move between velocities. Keep increasing until it overshoots and hten bring it down slightly.
     * Test by switching between TargetRPMs (BP did 274)
     */

    @Override
    public void run() {
        super.run();

        double F = launcher.getF();
        double P = launcher.getP();

        double error = launcher.getTargetRPM() - launcher.getLauncherRPM();
        // gamepad commands
        // target velocity, and update telemetry

        if (gamepad1.yWasPressed()) {
            if (launcher.getTargetRPM() == 0 ) {
                launcher.setTargetRPM(Constants.launcherConstants.CLOSE_ZONE_LAUNCH_RPM);
            } else if ((launcher.getTargetRPM() == Constants.launcherConstants.CLOSE_ZONE_LAUNCH_RPM) || (launcher.getTargetRPM() == Constants.launcherConstants.FAR_ZONE_LAUNCH_RPM)) {
                launcher.setTargetRPM(Constants.launcherConstants.MID_ZONE_LAUNCH_RPM);
            } else if (launcher.getTargetRPM() == Constants.launcherConstants.MID_ZONE_LAUNCH_RPM ) {
                launcher.setTargetRPM(Constants.launcherConstants.CLOSE_ZONE_LAUNCH_RPM);
            }
        }

        if (gamepad1.xWasPressed()) {
            if (launcher.getTargetRPM() == 0 ) {
                launcher.setTargetRPM(Constants.launcherConstants.MID_ZONE_LAUNCH_RPM);
            } else if ((launcher.getTargetRPM() == Constants.launcherConstants.CLOSE_ZONE_LAUNCH_RPM) || (launcher.getTargetRPM() == Constants.launcherConstants.FAR_ZONE_LAUNCH_RPM)) {
                launcher.setTargetRPM(Constants.launcherConstants.MID_ZONE_LAUNCH_RPM);
            } else if (launcher.getTargetRPM() == Constants.launcherConstants.MID_ZONE_LAUNCH_RPM ) {
                launcher.setTargetRPM(Constants.launcherConstants.FAR_ZONE_LAUNCH_RPM);
            }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            launcher.setF( F += stepSizes[stepIndex]);
        }

        if (gamepad1.dpadRightWasPressed()) {
            launcher.setF( F -= stepSizes[stepIndex]);
        }

        if (gamepad1.dpadUpWasPressed()) {
            launcher.setP( P += stepSizes[stepIndex]);
        }

        if (gamepad1.dpadDownWasPressed()) {
            launcher.setP( P -= stepSizes[stepIndex]);
        }

        launcher.applyPIDF();

        //launcher.printTelemetry(telemetry);
        telemetry.addLine("Y toggles between Close and Mid");
        telemetry.addLine("X toggles between Far and Mid");
        telemetry.addData("B changes step size", "%.4f", stepSizes[stepIndex]);
        telemetry.addData("F is dpad L/R", "%.4f", launcher.getF());
        telemetry.addData("P is dpad Up/Down","%.4f", launcher.getP());
        telemetry.addData("Target RPM", launcher.getTargetRPM());
        telemetry.addData("Current RPM", launcher.getLauncherRPM());
        telemetry.addData("Error", "%.2f", error);
        telemetry.update();

        packet.put("target_launcher_rpm", launcher.getTargetRPM());
        packet.put("current_launcher_rpm", launcher.getLauncherRPM());
        dash.sendTelemetryPacket(packet);

    }
}
