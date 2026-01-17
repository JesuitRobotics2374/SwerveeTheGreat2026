package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.Camera.Type;

public class VisionSubsystem extends SubsystemBase {

    /*
     * =====================================================
     * 0. VARIABLES
     * =====================================================
     */

    private Camera[] cameras; // An array holding the PhotonCamera instances

    private int numCams = Constants.numberOfCams; // The number of cameras on the robot
    private Transform3d[] cameraTransforms = { new Transform3d(0.375, -0.17, 0.115, new Rotation3d())
    };
    private Camera.Type[] types = { Type.APRIL_TAG };

    /*
     * =====================================================
     * 1. CONSTRUCTOR METHODS
     * =====================================================
     */

    /**
     * Constructor for the VisionSubsystem class.
     */
    public VisionSubsystem() {
        cameras = new Camera[numCams];

        for (int i = 0; i < numCams; i++) { // Initialize each camera
            System.out.println("initiazlied: " + types[i].toString() + "Camera" + i);

            cameras[i] = new Camera(types[i].toString() + "Camera" + i, cameraTransforms[i], types[i]);
        }
    }

    /*
     * =====================================================
     * 2. UTILITY / LIFECYCLE METHODS
     * =====================================================
     */

    /**
     * Updates the results from all cameras.
     */
    private void updateResults() {
        for (int i = 0; i < numCams; i++) { // Iterate through each camera
            cameras[i].updateResults(); // Update the results for the camera
        }
    }

    /**
     * Checks if all cameras are connected properly or not.
     * 
     * @return if every camera is connected or not.
     */
    public boolean getConnection() {
        for (int i = 0; i < numCams; i++) {
            if (cameras[i].getCameraType() == Camera.Type.DISCONNECTED) { // Intentionally disconnected cameras should
                                                                          // not affect the connection status
                continue;
            }

            if (!cameras[i].isConnected()) { // If any camera is not connected, return false
                return false;
            }
        }

        return true;
    }

    /**
     * Checks how many cameras of a certain type are connected.
     * 
     * @return the number of cameras of Type that are connected.
     */
    private int numCamsOfType(Camera.Type type) {
        int count = 0;

        for (int i = 0; i < numCams; i++) {
            if (cameras[i].getCameraType() != type) { // Skip cameras that are not of the specified type
                continue;
            }

            if (cameras[i].isConnected()) { // If the camera is connected, increment the count
                count++;
            }
        }

        return count; // Return the total count of connected cameras of the specified type
    }

    /*
     * =====================================================
     * 3. GLOBAL POSE
     * =====================================================
     */

    /**
     * Gets the global field poses estimated by all cameras.
     * NOTE: INCLUDES NULL VALUES FOR ESTIMATES
     * 
     * @return A list of EstimatedRobotPose objects from all cameras.
     */
    public List<PoseEstimateValues> getGlobalFieldPoses() {
        List<PoseEstimateValues> allEstimates = new ArrayList<>(); // A list to hold all estimated robot poses

        if (getConnection() == false || numCamsOfType(Camera.Type.APRIL_TAG) == 0) { // If no cameras are connected or
                                                                                     // no AprilTag cameras are present,
                                                                                     // return an empty list
            return allEstimates;
        }

        for (int i = 0; i < numCams; i++) { // Iterate through each camera
            if (cameras[i].getCameraType() != Camera.Type.APRIL_TAG) { // Skip cameras that are not AprilTag cameras
                continue;
            }

            allEstimates.add(cameras[i].getGlobalFieldPose()); // Add the estimated robot pose from the camera to the
                                                               // list
        }

        return allEstimates; // Return the list of all estimated robot poses
    }

    /*
     * =====================================================
     * 4. APRILTAG METHODS
     * =====================================================
     */

    /**
     * Gets a list of all visible tags from all cameras.
     * 
     * @return A list of all visible tag IDs.
     */
    public List<Integer> getAllVisibleTagIDs() {
        List<Integer> allTags = new ArrayList<>(); // A list to hold all visible tag IDs

        if (getConnection() == false || numCamsOfType(Camera.Type.APRIL_TAG) == 0) { // If no cameras are connected or
                                                                                     // no AprilTag cameras are present,
                                                                                     // return an empty list
            return allTags;
        }

        for (int i = 0; i < numCams; i++) { // Iterate through each camera and get its visible tags
            allTags.addAll(cameras[i].getAllAvailableTagIDs());
        }

        System.out.println(allTags.toString());

        return allTags; // Return the list of all visible tag IDs
    }

    /**
     * Checks if a specific tag is visible by any camera.
     * 
     * @param tagID - The ID of the tag to check, where -1 checks for any visible
     *              tag.
     * @return True if the tag is visible, false otherwise.
     */
    public boolean canSeeTag(int tagID) {
        List<Integer> allTags = getAllVisibleTagIDs(); // Get the list of all visible tags

        if (getConnection() == false || numCamsOfType(Camera.Type.APRIL_TAG) == 0) { // If no cameras are connected or
                                                                                     // no AprilTag cameras are present,
                                                                                     // return false
            return false;
        }

        if (tagID == -1) { // If tagID is -1, check if any tags are visible
            return allTags.size() > 0;
        }

        return allTags.contains(tagID); // Check if the specific tagID is in the list of visible tags
    }

    /**
     * Gets the pose of a specific tag relative to the robot, averaged across all
     * cameras.
     * 
     * @param tagID - The ID of the tag to get the pose for.
     * @return The averaged Pose3d of the tag relative to the robot.
     */
    public Pose3d getTagRelativeToBot(int tagID) {
        if (getConnection() == false || numCamsOfType(Camera.Type.APRIL_TAG) == 0) { // If no cameras are connected or
                                                                                     // no AprilTag cameras are present,
                                                                                     // return null
            return null;
        }

        ArrayList<Pose3d> tagPoses = new ArrayList<>(); // A list to hold the poses of the tag from each camera

        for (int i = 0; i < numCams; i++) { // Iterate through each camera

            if (cameras[i].getCameraType() != Camera.Type.APRIL_TAG) { // Skip cameras that are not AprilTag cameras
                continue;
            }

            tagPoses.add(cameras[i].getTagRelativeToBot(tagID)); // Add the tag pose from the camera to the list
        }

        return averagePoses(tagPoses); // Return the averaged tag pose
    }

    /**
     * Gets the distance to a specific tag from the robot.
     * 
     * @param tagID - The ID of the tag to get the distance to.
     * @return The distance to the tag in meters, or -1 if the tag is not visible.
     */
    public double getDistanceToTag(int tagID) {
        if (getConnection() == false || numCamsOfType(Camera.Type.APRIL_TAG) == 0) { // If no cameras are connected or
                                                                                     // no AprilTag cameras are present,
                                                                                     // return not a number
            return Double.NaN;
        }

        Pose3d tagPose = getTagRelativeToBot(tagID); // Get the pose of the tag relative to the robot

        if (tagPose == null) { // If the tag pose is not valid, return -1
            return -1;
        }

        return tagPose.getTranslation().getNorm(); // Return the distance to the tag
    }

    /**
     * Gets the nearest visible tag's pose relative to the robot.
     * 
     * @return The Pose3d of the nearest tag relative to the robot, or null if no
     *         tags are visible.
     */
    public Pose3d getNearestTag() {
        if (getConnection() == false || numCamsOfType(Camera.Type.APRIL_TAG) == 0) { // If no cameras are connected or
                                                                                     // no AprilTag cameras are present,
                                                                                     // return null
            return null;
        }

        List<Integer> visibleTags = getAllVisibleTagIDs(); // Get the list of all visible tags

        List<Pose3d> tagPoses = new ArrayList<>(); // A list to hold the poses of the visible tags

        for (int tagID : visibleTags) { // Iterate through each visible tag
            Pose3d tagPose = getTagRelativeToBot(tagID); // Get the pose of the tag relative to the robot

            if (tagPose != null) { // If the tag pose is valid, add it to the list
                tagPoses.add(tagPose);
            }
        }

        if (tagPoses.size() == 0) { // If no valid tag poses were found, return null
            return null;
        }

        Pose3d nearestTag = tagPoses.get(0); // Assume the first tag is the nearest initially

        for (Pose3d pose : tagPoses) { // Iterate through each tag pose to find the nearest one
            if (pose.getTranslation().getNorm() < nearestTag.getTranslation().getNorm()) { // If the current tag is
                                                                                           // closer than the nearest
                                                                                           // found so far
                nearestTag = pose; // Update the nearest tag
            }
        }

        return nearestTag; // Return the nearest tag pose
    }

    /*
     * =====================================================
     * 5. OBJECT DETECTION
     * =====================================================
     */

    /**
     * Get the nearest object of the specified type
     */
    public Pose3d getNearestObject(Camera.Type type) {
        if (getConnection() == false || numCamsOfType(type) == 0) { // If no cameras are connected or no AprilTag
                                                                    // cameras are present, return null
            return null;
        }

        ArrayList<Pose3d> poses = new ArrayList<>(); // Initialize the list of poses

        for (int i = 0; i < numCams; i++) { // Iterate through each camera

            if (cameras[i].getCameraType() != type) { // Skip cameras that are not of the specified type
                continue;
            }

            poses.addAll(cameras[i].getObjects(type)); // Add the poses from the camera to the list
        }

        if (poses.isEmpty()) { // If there are no valid poses, return null
            return null;
        }

        Pose3d closestPose = poses.get(0); // Set the first one as the closest pose
        double closestDistance = Math.sqrt(Math.pow(closestPose.getX(), 2)
                + Math.pow(closestPose.getY(), 2)); // Get its distance via the pythagorean theorem

        for (int i = 1; i < poses.size(); i++) { // Iterate through every pose besides the first one

            Pose3d tryPose = poses.get(i); // Get the new pose to compare to
            double tryPoseDistance = Math.sqrt(Math.pow(tryPose.getX(), 2)
                    + Math.pow(tryPose.getY(), 2)); // Get its distance via the pythagorean theorem

            if (tryPoseDistance < closestDistance) { // If the new distance is closer than the last closest
                closestDistance = tryPoseDistance; // Set the new closest distance
                closestPose = tryPose; // Update the best pose
            }
        }

        return closestPose; // Return the closest pose
    }

    /*
     * =====================================================
     * 6. POSE AVERAGING
     * =====================================================
     */

    /**
     * Averages a list of Pose3d objects.
     * 
     * @param poses - The list of Pose3d objects to average.
     * @return The averaged Pose3d object.
     */
    public Pose3d averagePoses(ArrayList<Pose3d> poses) {
        double x = 0;
        double y = 0;
        double z = 0;
        double rotX = 0;
        double rotY = 0;
        double rotZ = 0;
        double count = 0;

        for (Pose3d pose : poses) { // Iterate through each pose and sum the components
            if (pose == null) { // Skip null poses
                continue;
            }

            x += pose.getX();
            y += pose.getY();
            z += pose.getZ();
            rotX += pose.getRotation().getX();
            rotY += pose.getRotation().getY();
            rotZ += pose.getRotation().getZ();

            count++;
        }

        if (count == 0) { // If no valid poses were found, return null
            return null;
        }

        return new Pose3d(new Translation3d(x / count, y / count, z / count), // Return the averaged pose
                new Rotation3d(rotX / count, rotY / count, rotZ / count));
    }

    /*
     * =====================================================
     * 7. GETTERS / SETTERS
     * =====================================================
     */

    /*
     * =====================================================
     * 8. PERIODIC
     * =====================================================
     */

    private int clock;

    @Override
    public void periodic() {
        updateResults(); // Update camera results periodically

        clock++;

        if (clock >= 1) {
            clock = 0;
        }
    }
}