package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemV2;
import org.firstinspires.ftc.teamcode.util.MatchSettings;

@Disabled
@TeleOp (name = "LauncherPIDTest", group = "Tests")
public class LauncherPIDTest extends CommandOpMode {

    LauncherSubsystemV2 launcher;
    private FtcDashboard dash;
    TelemetryPacket packet = new TelemetryPacket();
    MatchSettings matchSettings;

    @Override
    public void initialize() {

        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        launcher = new LauncherSubsystemV2(hardwareMap, telemetry, matchSettings);

        telemetry.addLine("init done");
        telemetry.update();
    }

    // --- PIDF ---
    /**
     * Sets launcher PIDF coefficients manually in LauncherSubsystem Configs
     * Dashboard address - 192.168.43.1:8080/dash
     * kf - Set to a low value, just enough that the launcher wheel
     *           begins to rotate
     * kp - Increase kP after kF until the launcher wheel reaches the target speed
     * kd - Try changing the target speed of the launcher from a low value
     *           to a high value and vise versa. Use this to reduce the
     *           oscillations when changing speeds
     * ki - Most times this won't need to be tuned
     */

    @Override
    public void run() {
        super.run();

        launcher.applyPIDF();
        launcher.spinUp();


        packet.put("target_launcher_rpm", launcher.getTargetRPM());
        packet.put("current_launcher_rpm", launcher.getLauncherRPM());
        dash.sendTelemetryPacket(packet);

    }
}
