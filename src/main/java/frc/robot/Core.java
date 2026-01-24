// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Align.CanAlign;
import frc.robot.Align.DirectAlign;
import frc.robot.Align.ExactAlign;
import frc.robot.Align.FixYawToHub;
import frc.robot.Align.Target;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

public class Core {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.75; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveController = new CommandXboxController(0);

    public final DriveSubsystem drivetrain = TunerConstants.createDrivetrain();

    public final VisionSubsystem vision = new VisionSubsystem();

    private final double constantSlow = 0.25;

    private final Target testTarget = new Target(31, new Transform3d(1.69, 0.1, 0, new Rotation3d()));

    private final FixYawToHub fixYawToHub = new FixYawToHub(drivetrain, false);

    private final SequentialCommandGroup climbAlign = new SequentialCommandGroup(
            new DirectAlign(drivetrain, vision, testTarget),
            new CanAlign(drivetrain, vision, testTarget.requestFiducialID().get(), false));

    private boolean hubYawAlign = false;

     private static final double TranslationalAccelerationLimit = 10; // meters per second^2
    private static final double RotationalAccelerationLimit = Math.PI * 5.5; // radians per second^2

    private final SlewRateLimiter xRateLimiter = new SlewRateLimiter(TranslationalAccelerationLimit);
    private final SlewRateLimiter yRateLimiter = new SlewRateLimiter(TranslationalAccelerationLimit);
    private final SlewRateLimiter omegaRateLimiter = new SlewRateLimiter(RotationalAccelerationLimit);

    public Core() {
        configureBindings();
        // configureShuffleBoard();
    }

    public void configureShuffleBoard() {

        ShuffleboardTab tab = Shuffleboard.getTab("Test");

        // Limelight
        // HttpCamera httpCamera = new HttpCamera("Limelight",
        // "http://limelight.local:5800");
        // CameraServer.addCamera(httpCamera);
        // tab.add(httpCamera).withPosition(7, 0).withSize(3, 2);

        // New List Layout
        // ShuffleboardContainer pos = tab.getLayout("Position", "List
        // Layout").withPosition(0, 0).withSize(2, 3);

        // Field
        // tab.add(drivetrain.getField()).withPosition(2, 1).withSize(5, 3);

        // Modes
        // tab.addBoolean("Slow Mode", () -> isSlow()).withPosition(2, 0).withSize(2,
        // 1);
        // tab.addBoolean("Roll Mode", () -> isRoll()).withPosition(5, 0).withSize(1,
        // 1);

        // Robot (Reverse order for list layout)
        // pos.addDouble("Robot R", () -> drivetrain.getRobotR())
        // .withWidget("Gyro");
        // ;
        tab.addDouble("Robot Y", () -> drivetrain.getRobotY());
        // .withWidget("Number Bar");
        tab.addDouble("Robot X", () -> drivetrain.getRobotX());

        // tab.addBoolean("IN RANGE", () -> drivetrain.robotNearHP());

        // tab.add("Auto Chooser", autoChooser);

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                double axisScale = getAxisMovementScale();

                double driverVelocityX = driveController.getLeftY() * MaxSpeed * axisScale;
                double driverVelocityY = driveController.getLeftX() * MaxSpeed * axisScale;
                double driverRotationalRate = -driveController.getRightX() * MaxAngularRate * axisScale;

                // Determine which controller is active
                // boolean driverActive =
                //     Math.abs(driverVelocityX) > 0.05 ||
                //     Math.abs(driverVelocityY) > 0.05 ||
                //     Math.abs(driverRotationalRate) > 0.05;
                boolean driverActive = Math.abs(driveController.getRightX()) > 0.1 || !hubYawAlign;

                double desiredRotationalRate = driverActive ? driverRotationalRate : calculateRotationalRate();

                    return drive
                        .withVelocityX(xRateLimiter.calculate(-driverVelocityX)) // Limit translational acceleration forward/backward
                        .withVelocityY(yRateLimiter.calculate(-driverVelocityY)) // Limit translational acceleration left/right
                        .withRotationalRate(omegaRateLimiter.calculate(desiredRotationalRate));
            })

        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        // final var idle = new SwerveRequest.Idle();
        // RobotModeTriggers.disabled().whileTrue(
        // drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        // );

        // reset the field-centric heading on left bumper press
        driveController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driveController.a().onTrue(new DirectAlign(drivetrain, vision, testTarget));
        driveController.b().onTrue(new CanAlign(drivetrain, vision, testTarget.requestFiducialID().get(), false));

        driveController.x().onTrue(climbAlign);

        driveController.povUp().onTrue(new InstantCommand(() -> {
            fixYawToHub.schedule();
            hubYawAlign = true;
        }));
        driveController.povDown().onTrue(new InstantCommand(() -> {
            fixYawToHub.cancel(); 
            hubYawAlign = false;}));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public double getAxisMovementScale() {
        return (1 - (driveController.getRightTriggerAxis() * 0.75));
    }

    private double calculateRotationalRate() {
        return fixYawToHub.getRotationalRate();
    }
}
