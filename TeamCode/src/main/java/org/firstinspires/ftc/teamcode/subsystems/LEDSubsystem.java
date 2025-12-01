package org.firstinspires.ftc.teamcode.subsystems;

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

    ElapsedTime endGameTimer = new ElapsedTime();

    //Robot State Tracking
    private boolean intakingFront = false;
    private boolean intakingRear = false;
    private boolean intakingThru = false;
    private boolean intakeStopped = true;
    private boolean launcherAtSpeed = false;
    private boolean endGameSignaled = false;


    private static final Map<LedMode, RevBlinkinLedDriver.BlinkinPattern> modeLookup = new HashMap<>();

    // TEAM_COLORS, ALLIANCE_RED, ALLIANCE_BLUE,INTAKING_FRONT, INTAKING_REAR,INTAKING_THRU,
    // MOTIF_DETECTED, LAUNCHER_ATSPEED, GREEN_NEXT, PURPLE_NEXT, ELEVATING, BLACK

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
    }

    public LEDSubsystem(HardwareMap hardwareMap, MatchSettings matchSettings) {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        this.matchSettings = matchSettings;
        alliance = matchSettings.getAllianceColor();

        if (alliance == MatchSettings.AllianceColor.BLUE) {
            setMode(LedMode.ALLIANCE_BLUE);
        } else if (alliance == MatchSettings.AllianceColor.RED){
            setMode(LedMode.ALLIANCE_RED);
        } else {
            setMode(LedMode.TEAM_COLORS);
        }
    }

    public void update() {
        if (intakeStopped) {
            intakingFront = false;
            intakingRear = false;
            intakingThru = false;
        }

        if (endGameTimer.seconds() > 140 && endGameTimer.seconds() < 145 && !endGameSignaled) {
            setMode(LedMode.ELEVATING);
        } else if (endGameTimer.seconds() > 145 && !endGameSignaled) {
            setMode(LedMode.BLACK);
            endGameSignaled = true;
        }
        else if (intakingFront) {
            setMode(LedMode.INTAKING_FRONT);
        } else if (intakingRear) {
            setMode(LedMode.INTAKING_REAR);
        } else if (intakingThru) {
            setMode(LedMode.INTAKING_THRU);
        } else if (intakeStopped && launcherAtSpeed) {
            setMode(LedMode.LAUNCHER_ATSPEED);
        } else if (endGameTimer.seconds() < 10) {
            if (alliance == MatchSettings.AllianceColor.BLUE) {
                setMode(LedMode.ALLIANCE_BLUE);
            } else if (alliance == MatchSettings.AllianceColor.RED){
                setMode(LedMode.ALLIANCE_RED);
            } else {
                setMode(LedMode.TEAM_COLORS);
            }
        } else {
            setMode(LedMode.BLACK);
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

    public void startEndGameTimer() {
        endGameTimer.reset();
    }

    public void setTeamColors() {
        setMode(LedMode.TEAM_COLORS);
    }

    public void setAllianceRed () {
        setMode(LedMode.ALLIANCE_RED);
    }

    public void setAllianceBlue() {
        setMode(LedMode.ALLIANCE_BLUE);
    }

    public void setIntakingFront() {
        intakingFront = true;
    }

    public void setIntakingRear() {
        intakingRear = true;
    }

    public void setIntakingThru() {
        intakingThru = true;
    }

    public void setLauncherAtSpeed() {
        launcherAtSpeed = true;
    }

    public void setLauncherNotAtSpeed() {
        launcherAtSpeed = false;
    }

    public void setIntakingStopped() {
        intakeStopped = true;
    }
}
