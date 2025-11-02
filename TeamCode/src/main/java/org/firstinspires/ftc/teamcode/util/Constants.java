package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

public final class Constants {

    @Config
    public static final class DriveConstants{

    }

    @Config
    public static final class FieldConstants{

    }
    @Config
    public static final class intakeConstants{
        public static double INTAKE_POWER = 1;
        public static double REVERSE_INTAKE_POWER = -1;
        public static double DISTANCE_FOR_POSSESSION = 4; //TUNE!!!!!
    }

    @Config
    public static final class shooterConstants{
        public static int FAR_ZONE_SHOT_RPM = 5000;
        public static int MID_SHOT_RPM = 2500;
        public static int CLOSE_SHOT_RPM= 2300;
    }

    @Config
    public static class Util {
        public static double round(double in, int places) {
            return ((int) (in * Math.pow(10, places))) / (double) Math.pow(10, places);
        }
    }
}
