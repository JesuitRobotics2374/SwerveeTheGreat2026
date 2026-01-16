package frc.robot;

import java.nio.file.Paths;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;

public class Constants {
    public static int numberOfCams = 1;
    public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2025ReefscapeWelded;
    
    public static final double AUTO_X_OFFSET = 0;
    public static final double AUTO_ROLL_OFFSET = 0;
    public static final double AUTO_Y_OFFSET = 0;
    public static final double AUTO_PITCH_OFFSET = 0;
    public static final double AUTO_Z_OFFSET = 0;
    public static final double AUTO_YAW_OFFSET = 0;
}
