package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemV2;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemV3;
import org.firstinspires.ftc.teamcode.util.MatchSettings;

//@Disabled
@TeleOp (name = "LauncherPIDTest", group = "Tests")
public class LauncherPIDTest extends CommandOpMode {

    LauncherSubsystemV3 launcher;
    private FtcDashboard dash;
    TelemetryPacket packet = new TelemetryPacket();
    MatchSettings matchSettings;

    @Override
    public void initialize() {

        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        launcher = new LauncherSubsystemV3(hardwareMap, telemetry, matchSettings);

        telemetry.addLine("init done");
        telemetry.update();
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

        launcher.applyPIDF();
        launcher.printTelemetry(telemetry);

        packet.put("target_launcher_rpm", launcher.getTargetRPM());
        packet.put("current_launcher_rpm", launcher.getLauncherRPM());
        dash.sendTelemetryPacket(packet);

    }
}
