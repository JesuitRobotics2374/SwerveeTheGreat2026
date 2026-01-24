package frc.robot.Align;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.ArrayList;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

public class FixYawToHub extends Command {

    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // PID for Rotation
    private final PIDController yawController;

    // Rate limiter
    private final SlewRateLimiter yawRateLimiter = new SlewRateLimiter(6.0);

    // Position tolerance
    private static final double YAW_TOLERANCE = 5 * Math.PI / 180; // radians

    // Maximum output values
    private static final double MAX_ANGULAR_SPEED = 0.5;

    private static final double THETA_SPEED_MODIFIER = 0.75;

    // Minimum output to overcome static friction
    private static final double MIN_ANGULAR_COMMAND = 0.25;

    private final DriveSubsystem drivetrain;

    private Translation2d absoluteTargetTranslation;

    private double initialErrorYaw;

    private double dtheta;

    boolean finishedOverride;

    private Translation2d getAbsoluteTranslation(boolean isRed) {
        if (isRed) {
            return new Translation2d(); // TODO
        } else {
            return new Translation2d(4.625594, 4.034536);
        }
    }

    private double calculateRelativeTheta(Pose2d robotPose) {
        double delta_x = absoluteTargetTranslation.getX() - robotPose.getX();
        double delta_y = absoluteTargetTranslation.getY() - robotPose.getY();
        Rotation2d rotation = new Rotation2d(delta_x, delta_y);
        return rotation.getRadians();
    }

    public FixYawToHub(DriveSubsystem drivetrain, boolean isRed) {
        System.out.println("YAW LOCK CREATED");
        finishedOverride = false;

        this.drivetrain = drivetrain;

        this.absoluteTargetTranslation = getAbsoluteTranslation(isRed);

        this.initialErrorYaw = calculateRelativeTheta(drivetrain.getEstimator());

        // Yaw PID coefficients
        yawController = new PIDController(2.2, 0.1, 0.2);
        yawController.setTolerance(YAW_TOLERANCE);
        yawController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        System.out.println("YAW LOCK STARTED");

        // Reset controllers and rate limiters
        yawController.reset();
        yawRateLimiter.reset(0);

    }

    @Override
    public void execute() {

        drivetrain.setControl(driveRequest
                .withRotationalRate(-dtheta)
                );

        double error_yaw = calculateRelativeTheta(drivetrain.getEstimator());

        // Normalize yaw error to -π to π range
        error_yaw = Rotation2d.fromRadians(error_yaw).getRadians();

        // Calculate PID outputs
        dtheta = yawController.calculate(error_yaw, initialErrorYaw);

        // Apply minimum command if needed
        if (Math.abs(error_yaw) > YAW_TOLERANCE && Math.abs(dtheta) < MIN_ANGULAR_COMMAND) {
            dtheta = MIN_ANGULAR_COMMAND * Math.signum(dtheta);
        }

        // Limit output to maximum value
        dtheta = Math.max(-MAX_ANGULAR_SPEED, Math.min(dtheta * THETA_SPEED_MODIFIER, MAX_ANGULAR_SPEED));

        // Apply rate limiting for smoother motion
        dtheta = yawRateLimiter.calculate(dtheta);

        if (Math.abs(error_yaw) > 90 * Math.PI/180) {  
            System.out.println("(YL) yaw loop");
            error_yaw = Math.PI - Math.abs(error_yaw);
        }

    }

    @Override
    public boolean isFinished() {
        return false; // TODO
    }

    @Override
    public void end(boolean interrupted) {
        finishedOverride = true;
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        
        if (interrupted) {
            System.out.println("YAW LOCK INTERRUPTED");
        } else {
            System.out.println("YAW LOCK FINISHED");
        }
    }



}