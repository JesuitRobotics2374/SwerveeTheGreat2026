package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.Vision.PoseEstimateValues;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class DriveSubsystem extends TunerSwerveDrivetrain implements Subsystem {

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    // CUSTOM DECLARATIONS

    private final SwerveDrivePoseEstimator estimator;
    private List<PoseEstimateValues> estimates = new ArrayList<>();
    Field2d field = new Field2d();

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public DriveSubsystem(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);

        estimator = new SwerveDrivePoseEstimator(getKinematics(), getGyroscopeRotation(), getSwerveModulePositions(),
                new Pose2d());

        SmartDashboard.putData("Field", field);

        System.out.println("DriveSubsystem constructed");
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public DriveSubsystem(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);

        estimator = new SwerveDrivePoseEstimator(getKinematics(), getGyroscopeRotation(), getSwerveModulePositions(),
                new Pose2d());

        SmartDashboard.putData("Field", field);

        System.out.println("DriveSubsystem constructed");
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        estimator.update(getGyroscopeRotation(), getSwerveModulePositions());

        for (int i = 0; i < estimates.size(); i++) {
            PoseEstimateValues estimate = estimates.get(i);
            if (estimate != null) {
                addVisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, estimate.standardDeviations);
                estimator.addVisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, estimate.standardDeviations);
            } else {
                // System.out.println("Estimated pose NULL");
            }
        }

        field.setRobotPose(getState().Pose);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }

    private Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(getState().Pose.getRotation().getDegrees());
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] smp = new SwerveModulePosition[4];
        @SuppressWarnings("rawtypes")
        SwerveModule[] sms = getModules();
        for (int i = 0; i < 4; i++) {
            smp[i] = sms[i].getPosition(false);
        }
        return smp;
    }

    public void setRobotPose(Pose2d pose) {
        estimator.resetPose(pose);
    }

    public Field2d getField() {
        return field;
    }

    public double getRobotX() {
        return estimator.getEstimatedPosition().getX();
    }

    public double getRobotY() {
        return estimator.getEstimatedPosition().getY();
    }

    public double getRobotR() {
        return estimator.getEstimatedPosition().getRotation().getDegrees();
    }

    public Pose2d getEstimator() {
        return estimator.getEstimatedPosition();
    }

    public void setLabel(Pose2d pose2d, String label) {
        field.getObject(label).setPose(pose2d);
    }

    public void passGlobalEstimates(List<PoseEstimateValues> estimates) {
        this.estimates = estimates;
    }
}
