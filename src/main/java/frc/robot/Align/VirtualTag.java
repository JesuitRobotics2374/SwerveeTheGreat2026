package frc.robot.Align;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose3d;

public class VirtualTag {

    private Pose3d pose;

    private static HashMap<Integer, Pose3d> virtualTags = new HashMap<>();

    /**
     * Create a virtual tag with a specific ID and pose.
     * @param tagId The ID of the virtual tag.
     * @param pose The pose of the virtual tag.
     */
    public static void createVirtualTag(int tagId, Pose3d pose) {
        virtualTags.put(tagId, pose);
    }

    /**
     * Get the pose of a virtual tag by its ID.
     * @param tagId The ID of the virtual tag.
     * @return The pose of the virtual tag, or null if it does not exist.
     */
    public static Pose3d getVirtualTag(int tagId) {
        return virtualTags.get(tagId);
    }

    /**
     * Create a virtual tag at a specific pose.
     * @param pose The pose of the virtual tag.
     */
    public VirtualTag(Pose3d pose) {
        this.pose = pose;
    }

    /**
     * Get the pose of the virtual tag.
     * @return The pose of the virtual tag.
     */
    public Pose3d getPose() {
        return pose;
    }
    
}