package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemDemo;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemV3;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystemV3;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemV3;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.MatchSettings;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@TeleOp (name="Teleop14133_Demo", group = "Competition")
public class Teleop14133_Demo extends SampleCommandTeleop {

    public MatchSettings matchSettings;
    MatchSettings.AllianceColor alliance;

    private DriveSubsystemDemo drive;
    private LauncherSubsystemV3 launcher;
    private IntakeSubsystemV3 intake;
    private VisionSubsystem vision;
    private LEDSubsystem leds;

    boolean lockPrevPressed = false;
//    double allianceSteering;


    ElapsedTime localizationTimer;

    @Override
    public void onInit() {
        matchSettings = new MatchSettings(blackboard);
        MatchSettings.isAuto = false;
        MatchSettings.launcherState = MatchSettings.LauncherState.STOPPED;
        MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
        matchSettings.setAllianceColor(MatchSettings.AllianceColor.DEMO);
        alliance = matchSettings.getAllianceColor();

        drive = new DriveSubsystemDemo(hardwareMap, matchSettings);
        launcher = new LauncherSubsystemV3(hardwareMap, telemetry,matchSettings);
        intake = new IntakeSubsystemV3(hardwareMap, telemetry, matchSettings);
        vision = new VisionSubsystem(hardwareMap, matchSettings);
        leds = new LEDSubsystem(hardwareMap,matchSettings);

        localizationTimer = new ElapsedTime();

        boolean startVision = vision.isDetectingAGoalTag();  //Dummy to make the camera stream work in init??
    }

    @Override
    public void onStart() {

        //leds.startEndGameTimer();

    }

    @Override
    public void onLoop() {

        //launcher.printTelemetry(telemetry);
        intake.printTelemetry(telemetry);
        telemetry.update();

        runToggledDrive();

        intake.demoFSM(gamepad1);
        launcher.demoFSM();
        vision.teleopFSM();
        leds.update();


    }

    @Override
    public void onStop() {

    }

    public void runToggledDrive() {
        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x; // * 1.1; // ! ADJUST TURNING SENSITIVITY HERE ! //
        boolean yButton = gamepad1.y;

        drive.runManualMecanumDrive(forward, strafe, rotate, yButton);

    }
}
