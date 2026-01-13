package org.firstinspires.ftc.teamcode.subsystems;

import android.service.autofill.FieldClassification;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MatchSettings;

public class IntakeSubsystemV2 {

    // ================== SERVOS ================== \\
    private final CRServo frontIntake;
    private final CRServo frontMidIntake;
    private final CRServo rearMidIntake;
    private final CRServo rearIntake;
    private final CRServo leftTransfer;
    private final CRServo rightTransfer;

    // ================== SENSORS ================== \\
    private final RevColorSensorV3 frontColorSens;
    private final RevColorSensorV3 midColorSens;
    private final RevColorSensorV3 rearColorSens;
    private final DigitalChannel launcherBeamBreak;
//    private final DigitalChannel frontBeamBreak;
//    private final AnalogInput rearBeamBreak;

    double possessionDistanceFront = Constants.intakeConstants.DISTANCE_FOR_POSSESSION_FRONT;
    double possessionDistanceMid = Constants.intakeConstants.DISTANCE_FOR_POSSESSION_MID;
    double possessionDistanceRear = Constants.intakeConstants.DISTANCE_FOR_POSSESSION_REAR;
    double intaking = Constants.intakeConstants.INTAKE_POWER;
    double outtaking = Constants.intakeConstants.REVERSE_INTAKE_POWER;

    MatchSettings.ArtifactColor colorInSlot = MatchSettings.ArtifactColor.UNKNOWN;
    boolean possession = false; // Variable telling whether we have possession of a game piece or not

    // Possession and Color Variables for each intake round. Remember to reset!
    boolean possessionFront = false;
    boolean possessionMid = false;
    boolean possessionRear = false;

    boolean previousLauncherBB = false;
    boolean currentLauncherBB = false;

    String secondArtifactToLaunch = "TBD";
    String intakeLoadDirection = "TBD";
    int launchCounter = 0;
    MatchSettings.ArtifactColor secondArtifactNeeded = MatchSettings.ArtifactColor.UNKNOWN;

    private static final int RGB_COMPONENTS = 3;
    private final double[] rgbValues = new double[RGB_COMPONENTS];

    MatchSettings.ArtifactColor colorInSlotFront = MatchSettings.ArtifactColor.UNKNOWN;
    MatchSettings.ArtifactColor colorInSlotMid = MatchSettings.ArtifactColor.UNKNOWN;
    MatchSettings.ArtifactColor colorInSlotRear = MatchSettings.ArtifactColor.UNKNOWN;

    private static final double HSV_GREEN_MIN_H = 150;
    private static final double HSV_GREEN_MAX_H = 163;
    private static final double HSV_PURPLE_MIN_H = 165;
    private static final double HSV_PURPLE_MAX_H = 240;
    private static final double HSV_MIN_SAT = 0.3;
    private static final double HSV_MIN_VAL = 0.1;

    //=========== TELEMETRY ===========\\
    private final Telemetry telemetry;

    // CREATE MATCH SETTINGS / MOTIF ==============\\
    MatchSettings.Motif motif;

    public final MatchSettings matchSettings;

    //FSM tracker variables
    private boolean initialized = false;
    private ElapsedTime launchTimer = new ElapsedTime();

    private ElapsedTime autoTimer = new ElapsedTime();
    private double autoCancelSeconds = 28;

    public IntakeSubsystemV2(HardwareMap hardwareMap, Telemetry telemetry, MatchSettings matchSettings) {

        this.telemetry = telemetry;
        this.matchSettings = matchSettings;

        if (MatchSettings.isAuto) {
            colorInSlotFront = MatchSettings.ArtifactColor.PURPLE;
            colorInSlotMid = MatchSettings.ArtifactColor.PURPLE;
            colorInSlotRear = MatchSettings.ArtifactColor.GREEN;
        }

        // ================== SERVOS ================== \\
        frontIntake = hardwareMap.get(CRServo.class, "frontIntake");
        frontMidIntake = hardwareMap.get(CRServo.class, "frontMidIntake");
        rearMidIntake = hardwareMap.get(CRServo.class, "rearMidIntake");
        rearIntake = hardwareMap.get(CRServo.class, "rearIntake");
        leftTransfer = hardwareMap.get(CRServo.class, "leftTransfer");
        rightTransfer = hardwareMap.get(CRServo.class, "rightTransfer");

        frontIntake.setDirection(CRServo.Direction.REVERSE);
        frontMidIntake.setDirection(CRServo.Direction.REVERSE);
        rearMidIntake.setDirection(CRServo.Direction.FORWARD);
        rearIntake.setDirection(CRServo.Direction.FORWARD);
        leftTransfer.setDirection(CRServo.Direction.REVERSE);
        rightTransfer.setDirection(CRServo.Direction.FORWARD);

        // ================== SENSORS ================== \\
        frontColorSens = hardwareMap.get(RevColorSensorV3.class, "frontColor");
        midColorSens = hardwareMap.get(RevColorSensorV3.class, "midColor");
        rearColorSens = hardwareMap.get(RevColorSensorV3.class, "rearColor");

        frontColorSens.setGain(10);
        midColorSens.setGain(10);
        rearColorSens.setGain(10);

        launcherBeamBreak = hardwareMap.get(DigitalChannel.class, "launcherBB");
//        frontBeamBreak = hardwareMap.get(DigitalChannel.class, "frontBB");
//        rearBeamBreak = hardwareMap.get(AnalogInput.class, "rearBB");


    }

    public void teleopFSM(Gamepad gamepad2) {

        previousLauncherBB = currentLauncherBB;
        currentLauncherBB = !launcherBeamBreak.getState();

        //cancel intaking/transfer & STOP button
        if (gamepad2.dpad_right) {
            stopIntake();
            stopTransfer();
            initialized = false;
            MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
        }

        switch (MatchSettings.intakeState) {
            case STOPPED:
                initialized = false;
                // waiting for input
                if (gamepad2.dpad_up) {
                    MatchSettings.intakeState = MatchSettings.IntakeState.INTAKING_FRONT;
                } else if (gamepad2.dpad_down) {
                    MatchSettings.intakeState = MatchSettings.IntakeState.INTAKING_REAR;
                } else if (gamepad2.dpad_left) {
                    MatchSettings.intakeState = MatchSettings.IntakeState.INTAKING_THRU;
                } else if (gamepad2.right_bumper) {
                    MatchSettings.intakeState = MatchSettings.IntakeState.LAUNCHING_1_SIMPLE;
                }
                break;
            case INTAKING_FRONT:
                if (!initialized) { // powers on intake, if it is not on
                    clearIntakeLoadColors();
                    stopTransfer();
                    inboundFront(); // turn on all front intaking servos
                    inboundMidFront();
                    outboundMidRear();
                    inboundTransfer();
                    intakeLoadDirection = "FRONT";
                    initialized = true;
                }
                if (!rearPossession()) { //check for rear possession
                    // Check for color passing through Mid while waiting
                    if (colorInSlotRear == MatchSettings.ArtifactColor.UNKNOWN) {
                        colorInSlotRear = colorDetected(midColorSens);
                    }
                }
                if (rearPossession() && !midPossession()) {
                    stopMidRear();
                }
                if (rearPossession() && midPossession() && !frontPossession()) {
                    colorInSlotMid = colorDetected(midColorSens);
                    stopMidFront(); //otherwise stop checking, stop servo, and move on
                }
                if (rearPossession() && midPossession() && frontPossession()) {
                    stopIntake();
                    stopTransfer();
                    initialized = false;
                    MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
                }
                break;
            case INTAKING_REAR:
                if (!initialized) { // powers on intake, if it is not on
                    clearIntakeLoadColors();
                    stopTransfer();
                    inboundRear(); // turn on all rear intaking servos
                    inboundMidRear();
                    outboundMidFront();
                    inboundTransfer();
                    intakeLoadDirection = "REAR";
                    initialized = true;
                }
                if (!frontPossession()) { //check for front possession
                    // Check for color passing through Mid while waiting
                    if (colorInSlotFront == MatchSettings.ArtifactColor.UNKNOWN) {
                        colorInSlotFront = colorDetected(midColorSens);
                    }
                }
                if (frontPossession() && !midPossession()) {
                    stopMidFront();
                }
                if (frontPossession() && midPossession() && !rearPossession()) {
                    if (colorInSlotMid == MatchSettings.ArtifactColor.UNKNOWN) {
                    colorInSlotMid = colorDetected(midColorSens);}
                    stopMidRear(); //otherwise stop checking, stop servo, and move on
                }
                if (rearPossession() && midPossession() && frontPossession()) {
                    stopIntake();
                    stopTransfer();
                    initialized = false;
                    MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
                }
                break;
            case INTAKING_THRU:
                thruFrontAll();
                inboundTransfer();
                break;
            case LAUNCHING_1_SIMPLE:
                if (!initialized) {
                    initialized = true;
//                }
//                if (MatchSettings.launcherAtSpeed) {
                    inboundFront();
                    inboundMidFront();
                    inboundMidRear();
                    inboundRear();
                    outboundTransfer();
                }
                if (artifactLaunched() )
                        //|| !MatchSettings.launcherAtSpeed)
                {
                    stopIntake();
                    stopTransfer();
                    initialized = false;
                    MatchSettings.transferState = MatchSettings.TransferState.STOPPED;
                }
                break;
            default: // should never be reached
                MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
        }

//        switch (MatchSettings.transferState) {
//            case STOPPED:
//                if (gamepad2.left_bumper && MatchSettings.intakeState == MatchSettings.IntakeState.STOPPED && MatchSettings.launcherState == MatchSettings.LauncherState.SPINNING) {
//                    MatchSettings.transferState = MatchSettings.TransferState.LAUNCHING_3_SIMPLE;
//                } else if (gamepad2.right_bumper && MatchSettings.intakeState == MatchSettings.IntakeState.STOPPED && MatchSettings.launcherState == MatchSettings.LauncherState.SPINNING) {
//                    MatchSettings.transferState = MatchSettings.TransferState.LAUNCHING_1_SIMPLE;
//                }
//                break;
//            case LAUNCHING_3_SIMPLE:
//                if (gamepad2.dpad_right) {
//                    stopIntake();
//                    stopTransfer();
//                    initialized = false;
//                    MatchSettings.transferState = MatchSettings.TransferState.STOPPED;
//                    MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
//                } else {
//                    if (!initialized) {
//                        launchCounter = 0;
//                        secondArtifactNeeded = matchSettings.secondArtifactNeeded();
//                        initialized = true;
//                    }
//                    if (MatchSettings.launcherAtSpeed) {
//                        if (intakeLoadDirection == "FRONT" || colorInSlotRear == secondArtifactNeeded) {
//                            inboundRear();
//                        }else if(intakeLoadDirection == "REAR" || colorInSlotFront != secondArtifactNeeded){
//                            inboundRear();
//                        } else {
//                            inboundFront();
//                        }
//                        inboundMidFront();
//                        inboundMidRear();
//                        outboundTransfer();
//                    }
//
//                    if (launchCounter == 0 && (artifactLaunched() || !MatchSettings.launcherAtSpeed)) {
//                        launchTimer.reset();
//                        launchCounter++;
//                        stopTransfer(); //pause after 1st Artifact launched
//                    }
//                    if (launchCounter == 1 && (!artifactLaunched() || MatchSettings.launcherAtSpeed)) {
//                        outboundTransfer();
//                    }
//                    if (launchCounter == 1) {
//                        stopTransfer(); // pause after 2nd artifact launched
//                        inboundMidFront();
//                        inboundMidRear();
//                        inboundFront();
//                        inboundRear();
//                        launchTimer.reset();
//                        launchCounter++;
//                    }
//                    if (launchCounter == 2 && (!artifactLaunched() || MatchSettings.launcherAtSpeed)) {
//                        outboundTransfer();
//                    }
//                    if (launchCounter == 2) {
//                        stopTransfer(); //stop after 3rd Artifact launched
//                        stopIntake();
//                        launchCounter++;
//                    }
//                    if (launchCounter == 3) {
//                        initialized = false;
//                        MatchSettings.transferState = MatchSettings.TransferState.STOPPED;
//                    }
//                }
//                break;
//            case LAUNCHING_1_SIMPLE:
//                if (!initialized) {
//                    initialized = true;
//                }
//                if (MatchSettings.launcherAtSpeed) {
//                    inboundFront();
//                    inboundMidFront();
//                    inboundMidRear();
//                    inboundRear();
//                    outboundTransfer();
//                }
//                if (artifactLaunched() || !MatchSettings.launcherAtSpeed) {
//                    stopIntake();
//                    stopTransfer();
//                    initialized = false;
//                    MatchSettings.transferState = MatchSettings.TransferState.STOPPED;
//                }
//                break;
//            default: //should never be reached
//                MatchSettings.transferState = MatchSettings.TransferState.STOPPED;
//
//        }


    }

    //============== CONTROL METHODS ==============\\

    // COLOR SENSOR METHODS ==========================\\

    // Checks the launcher beam break to see if an artifact has passed through
    public boolean artifactLaunched() {
        if (previousLauncherBB && !currentLauncherBB) {
            return true;
        } else {
            return false;
        }
    }

    private MatchSettings.ArtifactColor colorDetected(RevColorSensorV3 sensor) { // Returns color detected if there is one
        NormalizedRGBA colors = sensor.getNormalizedColors();
        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(
                (int) (colors.red * 255),
                (int) (colors.green * 255),
                (int) (colors.blue * 255),
                hsv
        );
        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        boolean isGreen = (hue >= HSV_GREEN_MIN_H && hue <= HSV_GREEN_MAX_H);
        boolean isPurple = (hue >= HSV_PURPLE_MIN_H && hue <= HSV_PURPLE_MAX_H);
        boolean strongColor = sat >= HSV_MIN_SAT && val >= HSV_MIN_VAL;

        if (strongColor && isGreen) {
            return MatchSettings.ArtifactColor.GREEN;
        } else if (strongColor && isPurple) {
            return MatchSettings.ArtifactColor.PURPLE;
        } else {
            return MatchSettings.ArtifactColor.UNKNOWN;
        }
    }

    public void clearIntakeLoadColors() {
        colorInSlotFront = MatchSettings.ArtifactColor.UNKNOWN;
        colorInSlotMid = MatchSettings.ArtifactColor.UNKNOWN;
        colorInSlotRear = MatchSettings.ArtifactColor.UNKNOWN;
    }

    public void setRearColor() {
        colorInSlotRear = colorDetected(midColorSens);
    }

    public void setMidColor() {
        colorInSlotMid = colorDetected(midColorSens);
    }

    public void setFrontColor() {
        colorInSlotFront = colorDetected(midColorSens);
    }

    // DISTANCE SENSOR METHODS ==========================\\

    private boolean possessionDetectedFront(RevColorSensorV3 sensor) { //for bulk read before launching
        possession = sensor.getDistance(DistanceUnit.CM) < possessionDistanceFront;
        return possession;
    }

    private boolean possessionDetectedMid(RevColorSensorV3 sensor) { //for bulk read before launching
        possession = sensor.getDistance(DistanceUnit.CM) < possessionDistanceMid;
        return possession;
    }

    private boolean possessionDetectedRear(RevColorSensorV3 sensor) { //for bulk read before launching
        possession = sensor.getDistance(DistanceUnit.CM) < possessionDistanceRear;
        return possession;
    }


    public void readIntakePossessions() { // bulk read all sensors for initial possession values prior to launching
        possessionFront = possessionDetectedFront(frontColorSens);
        possessionMid = possessionDetectedMid(midColorSens);
        possessionRear = possessionDetectedRear(rearColorSens);
    }

    public void clearIntakePossessions() { // clear Intake load possession values
        possessionFront = false;
        possessionMid = false;
        possessionRear = false;
    }

    public boolean frontPossession() { // returns true if there is an artifact in distance
        possession = frontColorSens.getDistance(DistanceUnit.CM) < possessionDistanceFront;
        return possession;
    }

    public boolean midPossession() { // returns true if there is an artifact in distance
        possession = midColorSens.getDistance(DistanceUnit.CM) < possessionDistanceMid;
        return possession;
    }

    public boolean rearPossession() { // returns true if there is an artifact in distance
        possession = rearColorSens.getDistance(DistanceUnit.CM) < possessionDistanceFront;
       return possession;
//        return !rearBeamBreak.getState();
    }

    // IN-BOUND METHODS ==========================\\
    public void inboundFront() {
        frontIntake.setPower(intaking);
    }

    public void inboundMidFront() {
        frontMidIntake.setPower(intaking);
    }

    public void inboundMidRear() {
        rearMidIntake.setPower(intaking);
    }

    public void inboundRear() {
        rearIntake.setPower(intaking);
    }

    public void inboundTransfer() {
        leftTransfer.setPower(intaking);
        rightTransfer.setPower(intaking);
    }

    public void thruFrontAll() {
        frontIntake.setPower(intaking);
        frontMidIntake.setPower(intaking);
        rearMidIntake.setPower(outtaking);
        rearIntake.setPower(outtaking);
    }

    // OUT-BOUND METHODS ==========================\\
    public void outboundFront() {
        frontIntake.setPower(outtaking);
    }

    public void outboundMidFront() {
        frontMidIntake.setPower(outtaking);
    }

    public void outboundMidRear() {
        rearMidIntake.setPower(outtaking);
    }

    public void outboundRear() {
        rearIntake.setPower(outtaking);
    }

    public void outboundTransfer() {
        leftTransfer.setPower(outtaking);
        rightTransfer.setPower(outtaking);
    }

    // STOPPING METHODS ==========================\\
    public void stopFront() {
        frontIntake.setPower(0);
    }

    public void stopMidFront() {
        frontMidIntake.setPower(0);
    }

    public void stopMidRear() {
        rearMidIntake.setPower(0);
    }

    public void stopRear() {
        rearIntake.setPower(0);
    }

    public void stopTransfer() {
        leftTransfer.setPower(0);
        rightTransfer.setPower(0);
    }

    public void stopIntake() {
        frontIntake.setPower(0);
        frontMidIntake.setPower(0);
        rearMidIntake.setPower(0);
        rearIntake.setPower(0);
    }

    // TRANSFER TO LAUNCHER METHODS ==========================\\


    public void printTelemetry(Telemetry telemetry) { // Only for Testing. DO NOT constantly read sensors during teleop
        telemetry.addLine("INTAKE SUBSYSTEM");
        telemetry.addData("Front Sensor Distance", frontColorSens.getDistance(DistanceUnit.CM));
        telemetry.addData("Mid Sensor Distance", midColorSens.getDistance(DistanceUnit.CM));
        telemetry.addData("Rear Sensor Distance", rearColorSens.getDistance(DistanceUnit.CM));
        telemetry.addData("Front Color Detected", colorDetected(frontColorSens));
        telemetry.addData("Mid Color Detected", colorDetected(midColorSens));
        telemetry.addData("Rear Color Detected", colorDetected(rearColorSens));
        telemetry.addData("Launcher Beam Break", artifactLaunched());
//        telemetry.addData("Rear Beam Break New", rearBeamBreak.getVoltage());
        telemetry.update();
    }

    public MatchSettings getMatchSettings() {
        return matchSettings;
    }

    /**
     * ============== AUTONOMOUS ACTIONS ==============
     **/

    // Automatically intakes 3 artifacts from front. Times out after 5 seconds
    public class AutoIntake3Front implements Action {
        //check if initialized
        private boolean initialized = false;
        private ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on intake, if it is not on
            if (!initialized) { // turn on all front intaking servos
                MatchSettings.intakeState = MatchSettings.IntakeState.INTAKING_FRONT;
                clearIntakeLoadColors();
                inboundFront();
                inboundMidFront();
                outboundMidRear();
                timer.reset();
                initialized = true; //so that it skips this part next rerun
            }

            if (timer.time() > 2.25) { //stop intakes if it's been intaking longer than ## seconds
                stopIntake();
                MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
                return false;
            }

            if (!rearPossession()) { //check for rear possession
                // Check for color passing through Mid while waiting
                if (colorInSlotRear == MatchSettings.ArtifactColor.UNKNOWN) {
                    colorInSlotRear = colorDetected(midColorSens);
                }
                return true;  // rerun if sensor doesn't read possession
            } else {
                stopMidRear(); //otherwise stop checking, stop servo, and move on
            }

            if (!midPossession()) { //check for mid possession
                return true; //rerun if sensor doesn't read anything
            } else {
                colorInSlotMid = colorDetected(midColorSens);
                stopMidFront(); //otherwise stop checking, stop servo, and move on
            }

            if (!frontPossession()) { // check for front possession
                return true; // rerun if sensor doesn't read anything
            } else {
                stopFront(); //otherwise, turn off servo
                MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
                return false; //and return since action is complete
            }
        }
    }

    public Action autoIntake3Front() {
        return new AutoIntake3Front();
    }

    //---------------------------------------------------------------

    //temp for auto before the launcher is final - spits out balls from the intake for 3 seconds
    public class AutoSpitOut implements Action {
        private boolean initialized = false;
        private ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                thruFrontAll();
                timer.reset();
                initialized = true;
                return true;
            } else if (timer.seconds() < 2) {
                return true;
            } else {
                stopIntake();
                return false;
            }
        }
    }

    public Action autoSpitOut() {
        return new AutoSpitOut();
    }

    //---------------------------------------------------------------

    // Bulk reads for initial possession and colors. Launches Mid Artifact if there is one.
    public class AutoLaunch1st implements Action {
        // checks if the action has been initialized

        private boolean initialized = false;
        private boolean midIsLaunched = false;
        private ElapsedTime timerAction = new ElapsedTime();
        private ElapsedTime timerLaunch = new ElapsedTime();
        private MatchSettings.ArtifactColor desiredColor = matchSettings.secondArtifactNeeded();;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            previousLauncherBB = currentLauncherBB;
            currentLauncherBB = !launcherBeamBreak.getState();

            if (!initialized && autoTimer.seconds() < autoCancelSeconds) {
                // start by clearing possession values and then bulk read
                // for initial values for this intake load
                clearIntakePossessions();
                readIntakePossessions(); //saves initial possession values (lets us know how many Artifacts were loaded
                secondArtifactToLaunch = "TBD";
                MatchSettings.intakeState = MatchSettings.IntakeState.LAUNCHING_1_SIMPLE;

                // check for possession in midSlot and frontSlot and skip the rest of the action if nothing there
                // we need to know about the Front so that we can read the color there before moving on
//                if (!possessionMid) {
//                    return false;
//                }
//                else {
                    timerAction.reset();
                    inboundMidFront();
                    inboundMidRear();
                    outboundTransfer();
                    if(colorInSlotFront == desiredColor && colorInSlotRear !=desiredColor && desiredColor != MatchSettings.ArtifactColor.UNKNOWN){
                        inboundFront();
                        secondArtifactToLaunch = "Front";
                    } else { inboundRear(); secondArtifactToLaunch = "Rear"; }
                    initialized = true; //so that it skips this part next rerun
                    return true;
//                }
            } else if (timerAction.seconds() > 2 || autoTimer.seconds() >= autoCancelSeconds) {
                stopIntake();
                stopTransfer();
                MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
                return false; //if the whole thing is taking too long
            } else if (!artifactLaunched() && !midIsLaunched) {
                return true; //if the mid ball hasn't launched yet - RERUN
            } else if (artifactLaunched() && !midIsLaunched) { //skip next time
                midIsLaunched = true;
                stopTransfer();
                stopIntake();
                timerLaunch.reset();
                // once the artifact has gone through the launcher, stop servos
                return true;
//            } else if (midIsLaunched && timerLaunch.seconds() < 0.5) {
//                    return true;
            } else {
                MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
                return false;
            }
        }
    }
    public Action autoLaunch1st() {
        return new AutoLaunch1st();
    }

    //---------------------------------------------------------------

    // Launches a 2nd artifact. Chooses between front and rear depending on Motif
    public class AutoLaunch2nd implements Action {
        // checks if the action has been initialized
        private boolean initialized = false;
        private boolean launched = false;
        private ElapsedTime timerAction = new ElapsedTime();
        private String launchSlot = "TBD";

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            previousLauncherBB = currentLauncherBB;
            currentLauncherBB = !launcherBeamBreak.getState();

            if (!initialized && autoTimer.seconds() < autoCancelSeconds) {
                MatchSettings.intakeState = MatchSettings.IntakeState.LAUNCHING_1_SIMPLE;
//                if (!possessionFront && !possessionRear) { // check to see if there's an Artifact to launch
//                    MatchSettings.transferState = MatchSettings.TransferState.STOPPED;
//                    return false;
//                }
                if (secondArtifactToLaunch == "Front") { // choose a launch slot
                    launchSlot = "Front";
                } else {
                    launchSlot = "Rear";
                }
                timerAction.reset();
                // turn on front or rear servos depending on which artifact to be launched and where the front Artifact is
                if (launchSlot == "Front") {
                    inboundMidFront(); //send Front into launcher
                    inboundMidRear();
                    inboundFront();
                } else { //send Rear into launcher
                    inboundMidRear();
                    inboundMidFront();
                    inboundRear();
                }
                outboundTransfer();
                initialized = true; //so that it skips this part next rerun
                return true;
            } else if (timerAction.seconds() > 2 || autoTimer.seconds() >= autoCancelSeconds) {
                stopIntake();
                stopTransfer();
                MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
                return false; //if the whole thing is taking too long
            } else if (!launched) {
                if (artifactLaunched()) {
                    launched = true;
                    stopIntake();
                    stopTransfer();
                    launchTimer.reset();
                }
                return true;
//            } else if (launched && launchTimer.seconds() < 0.5) {
//                return true;
            } else {
                MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
                return false;
            }
        }
    }

    public Action autoLaunch2nd() {
        return new AutoLaunch2nd();
    }

    //---------------------------------------------------------------

    // Launches a 3rd artifact. Chooses between front and rear depending on where remaining Artifact is
    public class AutoLaunch3rd implements Action {
        // checks if the action has been initialized
        private boolean initialized = false;
        private ElapsedTime timerAction = new ElapsedTime();

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            previousLauncherBB = currentLauncherBB;
            currentLauncherBB = !launcherBeamBreak.getState();

            if (!initialized && autoTimer.seconds() < autoCancelSeconds) {
                MatchSettings.intakeState = MatchSettings.IntakeState.LAUNCHING_1_SIMPLE;
//                if (!possessionFront && !possessionRear) { // check to see if there's an Artifact to launch
//                    MatchSettings.transferState = MatchSettings.TransferState.STOPPED;
//                    return false;
//                }
                timerAction.reset();
                // turn on front and rear servos to feed whatever up
                inboundMidRear();
                inboundMidFront();
                inboundFront();
                inboundRear();
                outboundTransfer();
                initialized = true; //so that it skips this part next rerun
                return true;
            } else if (timerAction.seconds() > 2 || autoTimer.seconds() >= autoCancelSeconds) {
                stopIntake();
                stopTransfer();
                MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
                return false; //if the whole thing is taking too long
            } else if (!artifactLaunched()) {
                return true;
            } else {
                stopTransfer();
                stopIntake();
                MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
                return false;
            }
        }
    }
    public Action autoLaunch3rd() {
        return new AutoLaunch3rd();
    }

    //---------------------------------------------------------------

    //Sets slot colors based on which position on the field intaking from
    //Close to goal position
    public class AutoCloseColors implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            colorInSlotFront = MatchSettings.ArtifactColor.GREEN;
            colorInSlotMid = MatchSettings.ArtifactColor.PURPLE;
            colorInSlotRear = MatchSettings.ArtifactColor.PURPLE;
            return false;
        }
    }public Action autoCloseColors() { return new AutoCloseColors(); }

    //---------------------------------------------------------------

    //Sets slot colors based on which position on the field intaking from
    //Close to goal position
    public class AutoMidColors implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            colorInSlotFront = MatchSettings.ArtifactColor.PURPLE;
            colorInSlotMid = MatchSettings.ArtifactColor.GREEN;
            colorInSlotRear = MatchSettings.ArtifactColor.PURPLE;
            return false;
        }
    }public Action autoMidColors() { return new AutoMidColors(); }

    //Sets slot colors based on which position on the field intaking from
    //Close to goal position
    public class AutoFarColors implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            colorInSlotFront = MatchSettings.ArtifactColor.PURPLE;
            colorInSlotMid = MatchSettings.ArtifactColor.PURPLE;
            colorInSlotRear = MatchSettings.ArtifactColor.GREEN;
            return false;
        }
    }public Action autoFarColors() { return new AutoFarColors(); }

    //---------------------------------------------------------------

    //Sets slot colors based on which position on the field intaking from
    //Close to goal position
    public class AutoWallColors implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            colorInSlotFront = MatchSettings.ArtifactColor.PURPLE;
            colorInSlotMid = MatchSettings.ArtifactColor.GREEN;
            colorInSlotRear = MatchSettings.ArtifactColor.PURPLE;
            return false;
        }
    }public Action autoWallColors() { return new AutoWallColors(); }

    //---------------------------------------------------------------

    //Resets Auton Timer at the start of Auto - so that launching can be canceled

    public class AutoResetAutoTimer implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            autoTimer.reset();
            return false;
        }
    }public Action autoResetAutoTimer() { return new AutoResetAutoTimer(); }
}
