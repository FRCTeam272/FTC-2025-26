package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BaseRobot extends SubsystemBasePlus {

    public DriveSubsystem drive;
    public LauncherSubsystem launcher;
    public IntakeSubsystem intake;
    public Telemetry telemetry;

    public BaseRobot(HardwareMap hwmap, Pose2d startPos) {
        drive = new DriveSubsystem(hwmap, startPos);
        launcher = new LauncherSubsystem(hwmap, telemetry);
        intake = new IntakeSubsystem(hwmap,telemetry);

    }

    @Override
    public void printTelemetry(Telemetry t) {
        drive.printTelemetry(t);
        launcher.printTelemetry(t);
        intake.printTelemetry(t);

    }

    @Override
    public void periodic() {
        launcher.periodic();
    }

    public void stopAll() {
        launcher.stop();
        intake.stop();
    }
}
