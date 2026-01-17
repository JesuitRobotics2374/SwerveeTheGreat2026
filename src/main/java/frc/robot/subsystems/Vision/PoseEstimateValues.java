package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class PoseEstimateValues {
    public final Pose3d estimatedPose;
    public final double timestampSeconds;
    public final Matrix<N3, N1> standardDeviations;

    /**
     * Creates the class to hold these values
     * 
     * @param estimate - Estimated pose as per Vision
     * @param time     - Timestamp (seconds) of when the estimate was made
     * @param stdDev   - The standard deviations for the estimate
     */
    public PoseEstimateValues(Pose3d estimate, double time, Matrix<N3, N1> stdDev) {
        estimatedPose = estimate;
        timestampSeconds = time;
        standardDeviations = stdDev;
    }
}
