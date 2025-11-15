package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Constants;

public class IntakeSubsystem extends SubsystemBase {

    // ================== SERVOS ================== \\
    private final CRServo frontIntake;
    private final CRServo frontMidIntake;
    private final CRServo rearMidIntake;
    private final CRServo rearIntake;

    // ================== SENSORS ================== \\
    private final RevColorSensorV3 frontColorSens;
    private final RevColorSensorV3 midColorSens;
    private final RevColorSensorV3 rearColorSens;

    double possessionDistance = Constants.intakeConstants.DISTANCE_FOR_POSSESSION;
    double intaking = Constants.intakeConstants.INTAKE_POWER;
    double outtaking = Constants.intakeConstants.REVERSE_INTAKE_POWER;

    boolean possession = false; // Variable telling whether we have possession of a game piece or not

    private static final double HSV_GREEN_MIN_H = 60;
    private static final double HSV_GREEN_MAX_H = 180;
    private static final double HSV_PURPLE_MIN_H = 250;
    private static final double HSV_PURPLE_MAX_H = 320;
    private static final double HSV_MIN_SAT = 0.3;
    private static final double HSV_MIN_VAL = 0.1;

    //=========== TELEMETRY ===========\\
    private final Telemetry telemetry;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        // ================== SERVOS ================== \\
        frontIntake = hardwareMap.get(CRServo.class, "frontIntake");
        frontMidIntake = hardwareMap.get(CRServo.class, "frontMidIntake");
        rearMidIntake = hardwareMap.get(CRServo.class, "rearMidIntake");
        rearIntake = hardwareMap.get(CRServo.class, "rearIntake");

        frontIntake.setDirection(CRServo.Direction.REVERSE);
        frontMidIntake.setDirection(CRServo.Direction.REVERSE);
        rearMidIntake.setDirection(CRServo.Direction.FORWARD);
        rearIntake.setDirection(CRServo.Direction.FORWARD);

        // ================== SENSORS ================== \\
        frontColorSens = hardwareMap.get(RevColorSensorV3.class, "frontColor");
        midColorSens = hardwareMap.get(RevColorSensorV3.class, "midColor");
        rearColorSens = hardwareMap.get(RevColorSensorV3.class, "rearColor");

    }

    //============== CONTROL METHODS ==============\\

    // COLOR SENSOR METHODS ==========================\\

    private boolean isColorDetected(RevColorSensorV3 sensor) { // Returns true if detects Green or Purple
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

        return strongColor && (isGreen || isPurple);
    }

    private boolean isGreenDetected(RevColorSensorV3 sensor) { // Returns true if detects Green
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
        boolean strongColor = sat >= HSV_MIN_SAT && val >= HSV_MIN_VAL;

        return strongColor && isGreen;
    }

    private boolean isPurpleDetected(RevColorSensorV3 sensor) { // Returns true if detects Purple
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

        boolean isPurple = (hue >= HSV_PURPLE_MIN_H && hue <= HSV_PURPLE_MAX_H);
        boolean strongColor = sat >= HSV_MIN_SAT && val >= HSV_MIN_VAL;

        return strongColor && isPurple;
    }

    public boolean frontIsPurple() {
        return isPurpleDetected(frontColorSens);
    }

    public boolean midIsPurple() {
        return isPurpleDetected(midColorSens);
    }

    public boolean rearIsPurple() {
        return isPurpleDetected(rearColorSens);
    }

    public boolean frontIsGreen() {
        return isGreenDetected(frontColorSens);
    }

    public boolean midIsGreen() {
        return isGreenDetected(midColorSens);
    }

    public boolean rearIsGreen() {
        return isGreenDetected(rearColorSens);
    }

    // DISTANCE SENSOR METHODS ==========================\\

    public boolean frontPossession() { // returns true if there is an artifact in distance
        possession = frontColorSens.getDistance(DistanceUnit.CM) < possessionDistance;
        return possession;
    }

    public boolean midPossession() { // returns true if there is an artifact in distance
        possession = midColorSens.getDistance(DistanceUnit.CM) < possessionDistance;
        return possession;
    }

    public boolean rearPossession() { // returns true if there is an artifact in distance
        possession = rearColorSens.getDistance(DistanceUnit.CM) < possessionDistance;
        return possession;
    }

    public boolean notFrontPossession() { // returns true if there is not an artifact in distance
        possession = frontColorSens.getDistance(DistanceUnit.CM) < possessionDistance;
        return !possession;
    }

    public boolean notMidPossession() { // returns true if there is not an artifact in distance
        possession = midColorSens.getDistance(DistanceUnit.CM) < possessionDistance;
        return !possession;
    }

    public boolean notRearPossession() { // returns true if there is not an artifact in distance
        possession = rearColorSens.getDistance(DistanceUnit.CM) < possessionDistance;
        return !possession;
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

    public void intakeFromFront() {
        frontIntake.setPower(intaking);
        frontMidIntake.setPower(intaking);
        rearMidIntake.setPower(outtaking);
    }

    public void intakeFromRear() {
        frontMidIntake.setPower(outtaking);
        rearMidIntake.setPower(intaking);
        rearIntake.setPower(intaking);
    }

    public void inboundAll() {
        frontIntake.setPower(intaking);
        frontMidIntake.setPower(intaking);
        rearMidIntake.setPower(intaking);
        rearIntake.setPower(intaking);
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

    public void stop() {
        frontIntake.setPower(0);
        frontMidIntake.setPower(0);
        rearMidIntake.setPower(0);
        rearIntake.setPower(0);
    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addLine("INTAKE SUBSYSTEM");
        telemetry.addData("Front Sensor Distance", frontColorSens.getDistance(DistanceUnit.CM));
        telemetry.addData("Mid Sensor Distance", midColorSens.getDistance(DistanceUnit.CM));
        telemetry.addData("Rear Sensor Distance", rearColorSens.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}
