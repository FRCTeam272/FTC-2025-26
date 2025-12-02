package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
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

    double possessionDistanceFront = Constants.intakeConstants.DISTANCE_FOR_POSSESSION_FRONT;
    double possessionDistanceMid =  Constants.intakeConstants.DISTANCE_FOR_POSSESSION_MID;
    double possessionDistanceRear =  Constants.intakeConstants.DISTANCE_FOR_POSSESSION_REAR;
    double intaking = Constants.intakeConstants.INTAKE_POWER;
    double outtaking = Constants.intakeConstants.REVERSE_INTAKE_POWER;

    MatchSettings.ArtifactColor colorInSlot = MatchSettings.ArtifactColor.UNKNOWN;
    boolean possession = false; // Variable telling whether we have possession of a game piece or not

    // Possession and Color Variables for each intake round. Remember to reset!
    boolean possessionFront = false;
    boolean possessionMid = false;
    boolean possessionRear = false;

    String secondArtifactLaunched = "TBD";
    String intakeLoadDirection = "TBD";
    int launchCounter = 0;

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

    public IntakeSubsystemV2(HardwareMap hardwareMap, Telemetry telemetry, MatchSettings matchSettings) {

        this.telemetry = telemetry;
        this.matchSettings = matchSettings;

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
        leftTransfer.setDirection(CRServo.Direction.FORWARD);
        rightTransfer.setDirection(CRServo.Direction.REVERSE);

        // ================== SENSORS ================== \\
        frontColorSens = hardwareMap.get(RevColorSensorV3.class, "frontColor");
        midColorSens = hardwareMap.get(RevColorSensorV3.class, "midColor");
        rearColorSens = hardwareMap.get(RevColorSensorV3.class, "rearColor");

        frontColorSens.setGain(10);
        midColorSens.setGain(10);
        rearColorSens.setGain(10);

    }

    public void teleopFSM(Gamepad gamepad2) {
        switch (MatchSettings.intakeState) {
            case STOPPED:
                // waiting for input
                if (gamepad2.dpad_up) {
                    MatchSettings.intakeState = MatchSettings.IntakeState.INTAKING_FRONT;
                }
                else if (gamepad2.dpad_down) {
                    MatchSettings.intakeState = MatchSettings.IntakeState.INTAKING_REAR;
                }
                else if (gamepad2.dpad_left) {
                    MatchSettings.intakeState = MatchSettings.IntakeState.INTAKING_THRU;
                }
                break;
            case INTAKING_FRONT:
                if(!initialized) { // powers on intake, if it is not on
                    clearIntakeLoadColors();
                    inboundFront(); // turn on all front intaking servos
                    inboundMidFront();
                    outboundMidRear();
                    intakeLoadDirection = "FRONT";
                    initialized = true;
                }
                if(!rearPossession()) { //check for rear possession
                    // Check for color passing through Mid while waiting
                    if (colorInSlotRear == MatchSettings.ArtifactColor.UNKNOWN) {
                        colorInSlotRear = colorDetected(midColorSens);
                    }
                }
                if(rearPossession() && !midPossession()) {
                    stopMidRear();
                }
                if(rearPossession() && midPossession() && !frontPossession()) {
                    colorInSlotMid = colorDetected(midColorSens);
                    stopMidFront(); //otherwise stop checking, stop servo, and move on
                }
                if(rearPossession() && midPossession() && frontPossession()) {
                    stopFront();
                    initialized = false;
                    MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
                }
                break;
            case INTAKING_REAR:
                if(!initialized) { // powers on intake, if it is not on
                    clearIntakeLoadColors();
                    inboundRear(); // turn on all rear intaking servos
                    inboundMidRear();
                    outboundMidFront();
                    intakeLoadDirection = "REAR";
                    initialized = true;
                }
                if(!frontPossession()) { //check for front possession
                    // Check for color passing through Mid while waiting
                    if (colorInSlotFront == MatchSettings.ArtifactColor.UNKNOWN) {
                        colorInSlotFront = colorDetected(midColorSens);
                    }
                }
                if(frontPossession() && !midPossession()) {
                    stopMidFront();
                }
                if(frontPossession() && midPossession() && !rearPossession()) {
                    colorInSlotMid = colorDetected(midColorSens);
                    stopMidRear(); //otherwise stop checking, stop servo, and move on
                }
                if(rearPossession() && midPossession() && frontPossession()) {
                    stopRear();
                    initialized = false;
                    MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
                }
                break;
            case INTAKING_THRU:
                thruFrontAll();
                break;
            default: // should never be reached
                MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
        }
        //cancel intaking & STOP button
        if(gamepad2.dpad_right && MatchSettings.intakeState != MatchSettings.IntakeState.STOPPED) {
            stopAll();
            initialized = false;
            MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
        }

        switch(MatchSettings.transferState) {
            case STOPPED:
                if(gamepad2.right_bumper && MatchSettings.intakeState == MatchSettings.IntakeState.STOPPED && MatchSettings.launcherState == MatchSettings.LauncherState.SPINNING) {
                    MatchSettings.transferState = MatchSettings.TransferState.LAUNCHING_SIMPLE;
                }
                break;
            case LAUNCHING_SIMPLE:
                if(!initialized) {
                    launchTimer.reset();
                    launchCounter = 0;
                    initialized = true;
                }
                if (intakeLoadDirection == "FRONT"){
                    inboundMidRear();
                } else {
                    inboundMidFront();
                }
                outboundTransfer();
                if(launchCounter == 0 && launchTimer.seconds() > 1){
                    stopTransfer(); //pause after 1st Artifact launched
                    launchCounter++;
                }
                if(launchCounter == 1 && launchTimer.seconds() >1.5){
                    outboundTransfer();
                    launchTimer.reset();
                }
                if(launchCounter == 1 && launchTimer.seconds() > 1){
                    stopTransfer(); // pause after 2nd artifact launched
                    launchCounter++;
                    inboundMidFront();
                    inboundMidRear();
                }
                if(launchCounter == 2 && launchTimer.seconds() >1.5){
                    outboundTransfer();
                    launchTimer.reset();
                }
                if(launchCounter == 2 && launchTimer.seconds() > 1){
                    stopTransfer(); //stop after 3rd Artifact launched
                    stopMidRear();
                    stopMidFront();
                    launchCounter++;
                }
                if(launchCounter == 3 ){
                    MatchSettings.transferState= MatchSettings.TransferState.STOPPED;
                }
                break;
            default: //should never be reached
                MatchSettings.transferState= MatchSettings.TransferState.STOPPED;

        }
    }

    //============== CONTROL METHODS ==============\\

    // COLOR SENSOR METHODS ==========================\\

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

    public void stopAll() {
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
        telemetry.update();
    }

    /**============== AUTONOMOUS ACTIONS ==============**/

    // Automatically intakes 3 artifacts. Times out after 5 seconds
    public class AutoIntake3Front implements Action {
        //check if initialized
        private boolean initialized = false;
        private ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on intake, if it is not on
            if (!initialized) { // turn on all front intaking servos
                clearIntakeLoadColors();
                inboundFront();
                inboundMidFront();
                outboundMidRear();
                timer.reset();
                initialized = true; //so that it skips this part next rerun
            }

            if (timer.time() > 5) { //stop intakes if it's been intaking longer than 5 seconds
                stopAll();
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
                return false; //and return since action is complete
            }
        }
    }
    public Action autoIntake3Front() { return new AutoIntake3Front(); }

    //---------------------------------------------------------------

    // Bulk reads for initial possession and colors. Launches Mid Artifact if there is one.
    public class AutoLaunch1st implements Action {
        // checks if the action has been initialized

        private boolean initialized = false;
        private boolean midIsLaunched = false;
        private ElapsedTime timerAction = new ElapsedTime();
        private ElapsedTime timerTransfer = new ElapsedTime();

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if(!initialized) {
                // start by clearing possession values and then bulk read
                // for initial values for this intake load
                clearIntakePossessions();
                readIntakePossessions(); //saves initial possession values (lets us know how many Artifacts were loaded
                secondArtifactLaunched = "TBD";

                // check for possession in midSlot and frontSlot and skip the rest of the action if nothing there
                // we need to know about the Front so that we can read the color there before moving on
                if (!possessionMid) {
                    if (!possessionFront) {
                    return false;
                }
                timerAction.reset();
                inboundMidFront();
                inboundMidRear();
                outboundTransfer();
                initialized = true; //so that it skips this part next rerun
            }
            if(timerAction.time() > 2) {
                return false; //if the whole thing is taking too long
            }

            if(midPossession() && !midIsLaunched) {
                return true; //if the mid ball hasn't moved up yet - RERUN
                }
            } else if (!midIsLaunched) { //mid has moved up, nothing in front of sensor, skip next time
                midIsLaunched = true;
                timerTransfer.reset();
                // once the mid no longer detects an artifact OR its been long enough
                // stop intake servos, leave the transfer servos running to feed into launcher
                if (possessionFront) {
                    stopMidRear();
                } else { stopAll(); }
            }

            if (midPossession() && midIsLaunched) { //check to see if the front artifact has moved in front of the sensor, stop servo, store color
                stopMidFront();
                colorInSlotFront = colorDetected(midColorSens);
            }

            if(timerTransfer.time() < 0.5) {
                //tune - how long does it take to transfer once artifact is in transfer??
                //consider adding a beam break to determine if artifact has launched so it's not time based
                return true;
            } else {
                stopMidFront(); //just in case it's still running
                stopTransfer();
                return false;
            }
        }
    }
    public Action autoLaunch1st() { return new AutoLaunch1st(); }

    //---------------------------------------------------------------

    // Launches a 2nd artifact. Chooses between front and rear depending on Motif
    public class AutoLaunch2nd implements Action {
        // checks if the action has been initialized
        private boolean initialized = false;
        private ElapsedTime timer = new ElapsedTime();
        private String launchSlot = "TBD";
        private MatchSettings.Motif motif = null;
        private MatchSettings.ArtifactColor desiredColor = MatchSettings.ArtifactColor.UNKNOWN;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if(!initialized) {
                if(!possessionFront && !possessionRear) { // check to see if there's an Artifact to launch
                    return false;
                }
                if (!possessionMid) { // set desired color, 2nd motif slot unless there was no mid artifact
                    desiredColor = matchSettings.firstArtifactNeeded();
                } else {
                    desiredColor = matchSettings.secondArtifactNeeded();
                }
                if (possessionFront && colorInSlotFront == desiredColor) { // choose a launch slot
                    launchSlot = "Front";
                } else if (possessionRear && colorInSlotRear == desiredColor) {
                    launchSlot = "Rear";
                } else if (possessionFront) {
                    launchSlot = "Front";
                } else {
                    launchSlot = "Rear";
                }
                secondArtifactLaunched = launchSlot;
                timer.reset();
                // turn on front or rear servos depending on which artifact to be launched and where the front Artifact is
                if (launchSlot == "Front" && midPossession()) { //the front ball already there from checking color
                    inboundMidFront(); //send Front into launcher
                    inboundMidRear();
                } else if (launchSlot == "Front") { // it didn't get to the sensor? Push it along a little
                    inboundMidFront(); // send the Front Artifact toward middle without also pushing rear Artifact
                } else if (launchSlot == "Rear" && midPossession()) { // send the front ball backwards
                    outboundMidFront();
                } else if (launchSlot == "Rear" && !midPossession()) { //send Rear into launcher
                    inboundMidRear();
                }
                initialized = true; //so that it skips this part next rerun
            }
            if(timer.time() < 0.5) {
                return true;
            }
            inboundMidFront();
            inboundMidRear();
            outboundTransfer();
            if(timer.time() < 1) {
                return true;
            } else {
                stopTransfer();
                stopAll();
                return false;
            }
        }
    }
    public Action autoLaunch2nd() { return new AutoLaunch2nd(); }

    //---------------------------------------------------------------

    // Launches a 3rd artifact. Chooses between front and rear depending on where remaining Artifact is
    public class AutoLaunch3rd implements Action {
        // checks if the action has been initialized
        private boolean initialized = false;
        private ElapsedTime timer = new ElapsedTime();
        private String launchSlot = "TBD";

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if(!initialized) {
                if(!possessionFront && !possessionRear) { // check to see if there's an Artifact to launch
                    return false;
                }
                if (possessionRear && secondArtifactLaunched == "Front") {
                    launchSlot = "Rear";
                } else if (possessionFront && secondArtifactLaunched == "Rear"){
                    launchSlot = "Front";
                } else {
                    return false;
                }
                timer.reset();
                // turn on front or rear servos depending on which artifact to be launched
                if (launchSlot == "Rear") {
                    inboundMidRear();
                } else {
                    inboundMidFront();
                }
                initialized = true; //so that it skips this part next rerun
            }
            if(timer.time() < 0.5) {
                return true;
            }
            inboundMidFront();
            inboundMidRear();
            outboundTransfer();
            if(timer.time() < 1) {
                return true;
            } else {
                stopTransfer();
                stopAll();
                return false;
            }
        }
    }
    public Action autoLaunch3rd() { return new AutoLaunch3rd(); }

}
