// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Align;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;

public class Target {

    public enum Type {
        APRILTAG,
        FIELD
    }

    private Pose3d fieldPose;

    private Optional<Transform3d> fiducialOffset = Optional.empty();
    private Optional<Integer> fiducialID = Optional.empty();

    private Type type;

    /**
     * Create a field target at a specific pose.
     * @param pose The pose of the target on the field.
     */
    public Target(Pose3d pose) {
        fieldPose = pose;
        type = Type.FIELD;
    }

    /**
     * Create an AprilTag target with a fiducial ID and offset.
     * @param fiducialID The ID of the fiducial (tag).
     * @param fiducialOffset The offset of the target from the fiducial.
     */
    public Target(int fiducialID, Transform3d fiducialOffset) {
        this.fiducialID = Optional.of(fiducialID);
        this.fiducialOffset = Optional.of(fiducialOffset);
        this.fieldPose = calculateGlobalFieldPose(this, null);
        type = Type.APRILTAG;
    }

    /**
     * Calculate the global field pose of a target given its fiducial ID and offset.
     * @param target The target to calculate the pose for.
     * @param layout The AprilTag field layout.
     * @return The global field pose of the target, or null if it cannot be calculated.
     */
    private static Pose3d calculateGlobalFieldPose(Target target, AprilTagFieldLayout layout) {
        if (target.fiducialID.isPresent() && target.fiducialOffset.isPresent()) {
            try {
                if (layout == null ||
                    !layout.getTagPose(target.fiducialID.get()).isPresent()) {
                    Pose3d vt = VirtualTag.getVirtualTag(target.fiducialID.get());
                    if (vt != null) {
                        return vt.transformBy(
                            target.fiducialOffset.get());
                    }
                    return null;
                }
                var tagPose = layout.getTagPose(target.fiducialID.get());
                if (tagPose.isPresent()) {
                    return tagPose.get().transformBy(
                        target.fiducialOffset.get());
                }
            } catch (Exception e) {
                DriverStation.reportError(
                    "Unable to resolve AprilTag field pose.",
                    e.getStackTrace());
            }
        }
        return null;
    }

    /**
     * Get the field pose of the target.
     * @return The field pose of the target.
     */
    public Pose3d getFieldPose() {
        return fieldPose;
    }

    /**
     * Get the type of the target.
     * @return The type of the target.
     */
    public Type getType() {
        return type;
    }
    
    /**
     * Get the fiducial ID of the target, if it has one.
     * @return The fiducial ID of the target, or empty if it does not have one.
     */
    public Optional<Integer> requestFiducialID() {
        return fiducialID;
    }

    /**
     * Get the fiducial offset of the target, if it has one.
     * @return The fiducial offset of the target, or empty if it does not have one.
     */
    public Optional<Transform3d> requestFiducialOffset() {
        return fiducialOffset;
    }

    
}