package frc.robot.Align;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

public class CanAlign extends Command {
    private final DriveSubsystem drive;
    private final VisionSubsystem vision;
    private final CANrange range = new CANrange(27, "FastFD");
    
    private double initialRange;
    private final int tagToUse;

    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    
    private final double ySpeed = 1;
    private boolean goRight = true;
    private final double yFromOriginBeforeSlow = .3;

    public CanAlign(DriveSubsystem drive, VisionSubsystem vision, int tagToUse, boolean goRight) {
        System.out.println("CanAlign created");

        this.drive = drive;
        this.vision = vision;
        this.tagToUse = tagToUse;
        this.goRight = goRight;

        initialRange = range.getDistance().getValueAsDouble();

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        initialRange = range.getDistance().getValueAsDouble();
    }

    @Override
    public void execute() {
        double ySpeedMod = 1;

        if (goRight) {
            ySpeedMod *= -1;
        } else {
            ySpeedMod *= 1;
        }

        Pose3d poseToTag = vision.getTagRelativeToBot(tagToUse);
        
        if (poseToTag == null) {
            ySpeedMod *= 0.5;
        }
        else if (Math.abs(poseToTag.getY()) > yFromOriginBeforeSlow) {
            ySpeedMod *= .25;
        }
        else if (Math.abs(poseToTag.getY()) > yFromOriginBeforeSlow * 0.75) {
            ySpeedMod *= .75;
        }


        drive.setControl(driveRequest
                .withVelocityX(0)
                .withVelocityY(ySpeed * ySpeedMod)
                .withRotationalRate(0)
                );
    }

    @Override
    public boolean isFinished() {
        return range.getDistance().getValueAsDouble() <= initialRange/2;
    }

    @Override
    public void end(boolean interrupted) {
        drive.setControl(new SwerveRequest.SwerveDriveBrake());
        
        if (interrupted) {
            System.out.println("CanAlign INTERRUPTED");
        } else {
            System.out.println("CanAlign FINISHED");
        }
    }
}
