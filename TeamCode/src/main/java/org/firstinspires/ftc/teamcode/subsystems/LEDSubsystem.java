package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.LEDInterface;
import org.firstinspires.ftc.teamcode.util.MatchSettings;

import java.util.HashMap;
import java.util.Map;

public class LEDSubsystem implements LEDInterface{

    RevBlinkinLedDriver blinkin;
    LedMode currentMode;
    RevBlinkinLedDriver.BlinkinPattern currentPattern;

    public final MatchSettings matchSettings;
    MatchSettings.AllianceColor alliance;

    ElapsedTime signalTimer = new ElapsedTime();
    ElapsedTime endGameTimer = new ElapsedTime();
    private boolean endGameSignaled = false;
    private boolean signaled = false;
    private boolean isAuto = false;


    private static final Map<LedMode, RevBlinkinLedDriver.BlinkinPattern> modeLookup = new HashMap<>();

    static {
        modeLookup.put(LedMode.TEAM_COLORS, RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE);
        modeLookup.put(LedMode.ALLIANCE_RED, RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
        modeLookup.put(LedMode.ALLIANCE_BLUE, RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
        modeLookup.put(LedMode.INTAKING_FRONT, RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE );
        modeLookup.put(LedMode.INTAKING_REAR, RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);
        modeLookup.put(LedMode.INTAKING_THRU, RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
        modeLookup.put(LedMode.GREEN_NEXT, RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
        modeLookup.put(LedMode.PURPLE_NEXT, RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        modeLookup.put(LedMode.BLACK, RevBlinkinLedDriver.BlinkinPattern.BLACK);
        modeLookup.put(LedMode.ELEVATING, RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        modeLookup.put(LedMode.LAUNCHER_ATSPEED, RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        modeLookup.put(LedMode.MOTIF_DETECTED, RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES);
        modeLookup.put(LedMode.GOAL_DETECTED, RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE);
    }

    public LEDSubsystem(HardwareMap hardwareMap, MatchSettings matchSettings) {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        this.matchSettings = matchSettings;
        alliance = matchSettings.getAllianceColor();

        setAllianceColors();
    }

    public void update() {
        if (!MatchSettings.isAuto && !endGameSignaled && endGameTimer.seconds() > 140) { //only runs during teleop
            if (endGameTimer.seconds() < 143) {
                setMode(LedMode.ELEVATING);
            } else if (endGameTimer.seconds() > 143) {
                endGameSignaled = true;
                setMode(LedMode.BLACK);
            }
        }
        else if (MatchSettings.visionState == MatchSettings.VisionState.MOTIF_DETECTED){
            setMode(LedMode.MOTIF_DETECTED);
            if(!signaled) {
                signalTimer.reset();
                signaled = true;
            }
            else if (signalTimer.seconds() > 1) {
                MatchSettings.visionState = MatchSettings.VisionState.NONE;
                signaled = false;
            }
        }
        else if (MatchSettings.visionState == MatchSettings.VisionState.GOAL_DETECTED && MatchSettings.intakeState == MatchSettings.IntakeState.STOPPED) {
            setMode(LedMode.GOAL_DETECTED);
        }
        else if (MatchSettings.visionState == MatchSettings.VisionState.ARTIFACT_DETECTED) {
            setMode(LedMode.PURPLE_NEXT);
        }
        else if (MatchSettings.intakeState == MatchSettings.IntakeState.INTAKING_FRONT) {
            setMode(LedMode.INTAKING_FRONT);
        } else if (MatchSettings.intakeState == MatchSettings.IntakeState.INTAKING_REAR) {
            setMode(LedMode.INTAKING_REAR);
        } else if (MatchSettings.intakeState == MatchSettings.IntakeState.INTAKING_THRU) {
            setMode(LedMode.INTAKING_THRU);
        }
        //else if (MatchSettings.transferState == MatchSettings.TransferState.LAUNCHING_SIMPLE) {
//            setMode(LedMode.LAUNCHER_ATSPEED);
//        }
        else {
            setAllianceColors();
//        } else {
//            setMode(LedMode.BLACK);
        }

    }

    @Override
    public void setMode(LedMode mode) {
        if (modeLookup.containsKey(mode)) {
            blinkin.setPattern(modeLookup.get(mode));
            currentPattern = modeLookup.get(mode);
            currentMode = mode;
        } else {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            currentMode = LedMode.BLACK;
            currentPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        }
    }

    public LedMode getCurrentMode() {
        return currentMode;
    }

    public RevBlinkinLedDriver.BlinkinPattern getCurrentPattern() {
        return currentPattern;
    }

    public void setAllianceColors() {
        if (alliance == MatchSettings.AllianceColor.BLUE) {
            setMode(LedMode.ALLIANCE_BLUE);
        } else if (alliance == MatchSettings.AllianceColor.RED){
            setMode(LedMode.ALLIANCE_RED);
        } else {
            setMode(LedMode.TEAM_COLORS);
        }
    }

    public void startEndGameTimer() {
        endGameTimer.reset();
    }

    //============== AUTONOMOUS ACTIONS ==============\\

    //follower for LEDs
    public class UpdateAuto implements Action {
        @Override
        public boolean run (@NonNull TelemetryPacket packet) {
            update();
            return true;
        }
    }
    public Action updateAuto() { return new UpdateAuto(); }
}
