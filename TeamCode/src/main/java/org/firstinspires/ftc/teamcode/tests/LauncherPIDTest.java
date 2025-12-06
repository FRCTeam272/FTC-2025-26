package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

@Disabled
@TeleOp (name = "LauncherPIDTest", group = "Tests")
public class LauncherPIDTest extends CommandOpMode {

    LauncherSubsystem launcher;
    private FtcDashboard dash;
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void initialize() {

        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        launcher = new LauncherSubsystem(hardwareMap, telemetry);

        telemetry.addLine("init done");
        telemetry.update();
    }

    // --- PIDF ---
    /**
     * Sets shooter PIDF coefficients manually in LauncherSubsystem Configs
     * Dashboard address - 192.168.43.1:8080/dash
     * kf - Set to a low value, just enough that the shooter wheel
     *           begins to rotate
     * kp - Increase kP after kF until the shooter wheel reaches the target speed
     * kd - Try changing the target speed of the shooter from a low value
     *           to a high value and vise versa. Use this to reduce the
     *           oscillations when changing speeds
     * ki - Most times this won't need to be tuned
     */

    @Override
    public void run() {
        super.run();

        launcher.applyPIDF();
        launcher.spinUp();


        packet.put("target_shooter_rpm", launcher.getTargetRPM());
        packet.put("current_shooter_rpm", launcher.getLauncherRPM());
        dash.sendTelemetryPacket(packet);

    }
}
