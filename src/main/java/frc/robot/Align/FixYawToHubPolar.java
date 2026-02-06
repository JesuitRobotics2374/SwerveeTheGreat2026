package frc.robot.Align;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;

public class FixYawToHubPolar extends Command {

    private final DriveSubsystem drivetrain;
    private final CommandXboxController controller;
    private final Translation2d hub;

    private final PIDController yawController =
        new PIDController(5.0, 0.7, 2.5);

    private final SlewRateLimiter omegaLimiter =
        new SlewRateLimiter(6.0);

    private static final double MAX_R_SPEED = 3.0;       // m/s
    private static final double MAX_TANG_SPEED = 2.5;   // rad/s
    private static final double MAX_OMEGA = 2.0;

    private ChassisSpeeds output = new ChassisSpeeds();

    public FixYawToHubPolar(
            DriveSubsystem drivetrain,
            boolean isRed) {

        this.drivetrain = drivetrain;
        this.controller = new CommandXboxController(0);

        this.hub = isRed
                ? new Translation2d() // TODO
                : new Translation2d(4.625594, 4.034536);

        yawController.enableContinuousInput(-Math.PI, Math.PI);
        yawController.setTolerance(Math.toRadians(3));

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

        Pose2d pose = drivetrain.getState().Pose;

        /* =======================
           Polar geometry
           ======================= */
        double dx = pose.getX() - hub.getX();
        double dy = pose.getY() - hub.getY();

        double r = Math.hypot(dx, dy);
        double phi = Math.atan2(dy, dx);

        Translation2d radial =
            new Translation2d(Math.cos(phi), Math.sin(phi));

        Translation2d tangential =
            new Translation2d(-Math.sin(phi), Math.cos(phi));

        /* =======================
           Driver inputs
           ======================= */
        double rCmd = -controller.getLeftY() * MAX_R_SPEED;
        double phiCmd = -controller.getLeftX() * MAX_TANG_SPEED;

        /* =======================
           Field-relative velocity
           ======================= */
        Translation2d vField =
            radial.times(rCmd)
                  .plus(tangential.times(r * phiCmd));

        /* =======================
           Yaw lock to hub
           ======================= */
        Rotation2d desiredYaw =
            new Rotation2d(Math.atan2(-dy, -dx));

        double omega =
            yawController.calculate(
                pose.getRotation().getRadians(),
                desiredYaw.getRadians());

        omega = Math.max(-MAX_OMEGA, Math.min(omega, MAX_OMEGA));
        omega = omegaLimiter.calculate(omega);

        /* =======================
           Output chassis speeds
           ======================= */
        output = ChassisSpeeds.fromFieldRelativeSpeeds(
            vField.getX(),
            vField.getY(),
            omega,
            pose.getRotation()
        );
    }

    public ChassisSpeeds getChassisSpeeds() {
        return output;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
