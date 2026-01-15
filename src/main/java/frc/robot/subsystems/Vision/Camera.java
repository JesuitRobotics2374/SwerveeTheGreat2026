package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class Camera {
    private PhotonCamera camera; // The PhotonCamera instance
    private String cameraName; // The name of the camera
    private Type type; // The type of object the camera detects
    private PhotonPoseEstimator poseEstimator; // The pose estimator for AprilTag cameras
    private Transform3d robotToCameraTransform; // The transform from the robot to the camera
    private PhotonPipelineResult latestResult; // The latest result from the camera

    // Camera constants - FOR THE COLOR CAM ONLY
    private static final double IMAGE_WIDTH = 640.0;
    private static final double IMAGE_HEIGHT = 480.0;
    private static final double VERTICAL_FOV_DEG = 47.72;

    // Camera intrinsics from calibration - FOR THE COLOR CAM ONLY
    private static final double FX = 545.08; // pixels
    private static final double FY = 542.63; // pixels
    private static final double CX = 349.47; // pixels (unused for pinhole model here)
    private static final double CY = 221.75; // pixels (unused for pinhole model here)

    // Constants for photon error - FOR OD ONLY
    private static final double xFix = 2;
    private static final double yFix = 2;
    private static final double zFix = 2;

    // Physical object size (pool noodle OD = 2.5 in)
    private static final double OBJECT_HEIGHT_M = 2.5 * 0.0254; // 0.0635 m

    // Enum for the type of object the camera detects
    public enum Type {
        APRIL_TAG,
        OBJECT,
        DISCONNECTED // Skips the camera in all logic
    }

    /**
     * Constructor for the Camera class.
     * 
     * @param nti                    - the NetworkTableInstance of the robot
     * @param cameraName             - the name of the camera in PhotonVision
     * @param robotToCameraTransform - the Transform3d from the robot to the camera
     */
    public Camera(String cameraName, Transform3d robotToCameraTransform, Type type) {
        System.out.println(NetworkTableInstance.getDefault());

        this.cameraName = cameraName; // Store the name
        this.robotToCameraTransform = robotToCameraTransform; // Store the robot-to-camera transform
        this.type = type; // Set the type based on the given type

        this.camera = new PhotonCamera(NetworkTableInstance.getDefault(), this.cameraName); // Initialize the
                                                                                            // PhotonCamera
        // with the given NTI and name

        if (this.type == Type.APRIL_TAG) { // If the type is for AprilTags, initialize the pose estimator
            AprilTagFieldLayout aprilTagFieldLayout = loadField(); // Load the AprilTag field layout

            poseEstimator = new PhotonPoseEstimator( // Create a new PhotonPoseEstimator with the field layout and
                                                     // robot-to-camera transform
                    aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, // This is default, can change
                    robotToCameraTransform);

            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); // Set the fallback strategy for
                                                                                      // multi-tag estimation
        } else { // If the type is not for AprilTags, set the pose estimator to null
            poseEstimator = null;
        }
    }

    /**
     * Loads the AprilTag field layout from the file.
     */
    private AprilTagFieldLayout loadField() {
        try { // Load the AprilTag field layout from the file
            return Constants.FIELD_LAYOUT.loadAprilTagLayoutField();
        } catch (Exception e) { // Handle any exceptions that occur during loading
            e.printStackTrace();
        }

        return null; // Return null if loading fails
    }

    /**
     * Updates the camera to store its latest results for use in methods.
     */
    public void updateResults() {
        List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults(); // Get all unread results from the
                                                                                 // camera

        if (!isConnected()) {
            latestResult = null;
            return;
        }

        if (unreadResults.size() == 0) { // If there are no unread results, ignore changing anything
            return;
        } else {
            latestResult = unreadResults.get(0); // Update latestResult to the most recent unread result
        }
    }

    /**
     * Returns whether the camera is connected or not. 
     * @return whether the camera is connected or not.
     */
    public boolean isConnected() {
        return camera.isConnected(); // Return if the camera is connected or not
    }

    /**
     * Adjust the raw pose obtained from the camera to account for the camera's
     * position and orientation on the robot.
     * 
     * @param rawPose - the raw Pose3d obtained from the camera.
     * @return Adjusted Pose3d representing the robot's pose on the field.
     */
    private Pose3d adjustPose(Transform3d rawPose) {
        if (rawPose == null) { // If the raw pose is null, return null
            return null;
        }

        if (type == Type.APRIL_TAG) { // if it is an aprilTag value, normalize the yaw
            rawPose = normalizeYaw(rawPose);
        }

        if (robotToCameraTransform.getRotation().getY() != 0) {
            double pitch = robotToCameraTransform.getRotation().getY(); // get the pitch
            double hypotenuse = rawPose.getTranslation().getX();

            double XPitchFix = hypotenuse * Math.cos(pitch); // fix the forward distance
            double YPitchFix = rawPose.getTranslation().getY(); // no need to fix anything
            double ZPitchFix = hypotenuse * Math.sin(pitch); // fix the vertical distance

            rawPose = new Transform3d(XPitchFix, YPitchFix, ZPitchFix, rawPose.getRotation());
        }

        Transform3d adjustedPose = rawPose.plus(robotToCameraTransform); // Adjust the raw pose by adding the
                                                                         // robot-to-camera transform, TODO: CHECK IF
                                                                         // THIS NEEDS TO BE SUBTRACTED

        return new Pose3d(adjustedPose.getTranslation(), adjustedPose.getRotation()); // Return the adjusted pose as a
                                                                                      // Pose3d
    }

    /**
     * Normalizes the yaw of the given transform3d so that the April Tag values
     * given by Photon do not have 180° as centered
     * 
     * @param rawTransform - The raw, not normalized transform3d
     * @return The normalized transform3d
     */
    private Transform3d normalizeYaw(Transform3d rawTransform) {
        double yawValue = rawTransform.getRotation().getZ();

        if (Math.abs(yawValue) > 90 * Math.PI/180) { 
            yawValue = Math.PI - Math.abs(yawValue) * -1 * (yawValue/Math.abs(yawValue));
        }
        else {
            return rawTransform;
        }

        Rotation3d rotation = rawTransform.getRotation();


        return new Transform3d(rawTransform.getTranslation(),
                               new Rotation3d(rotation.getX(), rotation.getY(), yawValue));
    }

    /**
     * Get the global field pose of the robot as estimated by the camera.
     * 
     * @return EstimatedRobotPose object containing the robot's pose and targets
     *         used to find this, or null if no valid pose is available.
     */
    public EstimatedRobotPose getGlobalFieldPose() {
        if (!type.equals(Type.APRIL_TAG) || !isConnected()) { // If the type is not for an AprilTag, return null
            return null;
        }

        if (latestResult == null || !latestResult.hasTargets()) { // If there are no targets in the latest result,
                                                                  // return null
            return null;
        }

        Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update(latestResult); // Estimate the robot's
                                                                                              // pose using the latest
                                                                                              // result

        if (estimatedRobotPose.isPresent()) { // If a valid pose is estimated, return it
            return estimatedRobotPose.get();
        }

        return null; // Return null if no valid pose is estimated
    }

    /**
     * Get a list of all available AprilTag IDs detected by the camera.
     * 
     * @return ArrayList of Integer tag IDs.
     */
    public ArrayList<Integer> getAllAvailableTagIDs() {
        ArrayList<Integer> tagIDs = new ArrayList<Integer>(); // Initialize an empty list to store tag IDs

        if (!type.equals(Type.APRIL_TAG) || !isConnected()) { // If the type is not for an AprilTag, return null
            return tagIDs;
        }

        if (latestResult == null || !latestResult.hasTargets()) { // If there are no targets in the latest result,
                                                                  // return null
            return tagIDs;
        }

        for (PhotonTrackedTarget target : latestResult.getTargets()) { // Iterate through each detected target
            tagIDs.add(target.getFiducialId()); // Add the tag ID to the list
        }

        return tagIDs; // Return the list of tag IDs
    }

    /**
     * Get the pose of a specific AprilTag relative to the robot.
     * 
     * @param tagID - the ID of the AprilTag to find.
     * @return Transform3d of the specified tag relative to the robot, or null if
     *         not found.
     */
    public Pose3d getTagRelativeToBot(int tagID) {
        if (!type.equals(Type.APRIL_TAG) || !isConnected()) { // If the type is not for an AprilTag, return null
            return null;
        }

        if (latestResult == null || !latestResult.hasTargets()) { // If there are no targets in the latest result,
                                                                  // return null
            return null;
        }

        for (PhotonTrackedTarget target : latestResult.getTargets()) { // Iterate through each detected target
            if (target.getFiducialId() == tagID) { // If the target's ID matches the requested tagID
                Transform3d cameraToTarget = target.getBestCameraToTarget(); // Get the transform from the camera to the
                                                                             // target

                return adjustPose(cameraToTarget); // Return the pose of the tag relative to the robot
            }
        }

        return null; // Return null if the specified tagID is not found
    }

    /**
     * DEPRACATED UNTIL FURTHER NOTICE, DOES NOT WORK
     * Get the pose of a specific AprilTag relative to the robot.
     * 
     * @param tagID - the ID of the AprilTag to find.
     * @return Pose3d of the specified tag relative to the robot, or null if not
     *         found.
     */
    public Pose3d getBotRelativeToTag(int tagID) {
        // Pose3d tagRelativeToBot = getTagRelativeToBot(tagID); // Get the pose of the
        // tag relative to the robot

        // if (tagRelativeToBot == null) { // If the tag relative to bot is null, return
        // null
        // return null;
        // }

        // Transform3d botRelativeToTag = new
        // Transform3d(tagRelativeToBot.getTranslation(),
        // tagRelativeToBot.getRotation()).inverse(); // Invert the transform to get the
        // robot relative to the tag

        // return new Pose3d(botRelativeToTag.getTranslation(),
        // botRelativeToTag.getRotation()); // Return the pose of the robot relative to
        // the tag
        return null;
    }

    /**
     * Get a list of all of the specific object poses of the type passed
     * 
     * @param type - the type to look for
     * @return a list of all specific object poses of the type passed
     */
    public List<Pose3d> getObjects(Type type) {
        ArrayList<Pose3d> poses = new ArrayList<>(); // Initialize the list

        if (this.type != Type.OBJECT || !isConnected()) { // If the type does not match, return the empty list
            return poses;
        }

        if (latestResult == null || !latestResult.hasTargets()) { // If there are no targets in the latest result,
                                                                  // return the empty list
            return poses;
        }

        for (PhotonTrackedTarget target : latestResult.getTargets()) { // Iterate through each target and add it
            Pose3d pose = adjustPose(calculateObjectPose(target.getArea(), target.getYaw(), target.getPitch()));

            if (pose == null) { // Skip null poses
                continue;
            }

            poses.add(pose);
        }

        return poses; // Return the list of poses
    }

    /**
     * Calculates the 3D pose of the object relative to the camera using area
     * percent.
     *
     * @param areaPercent - Percent of the total image area the object occupies
     *                    (0-100)
     * @param yawDeg      - Horizontal angle from PhotonVision (degrees)
     * @param pitchDeg    - Vertical angle from PhotonVision (degrees)
     * @param imageWidth  - Camera image width (pixels)
     * @param imageHeight - Camera image height (pixels)
     * @return Transform3d of the object in the camera frame
     */
    public static Transform3d calculateObjectPose(double areaPercent, double yawDeg, double pitchDeg) {
        if (areaPercent <= 0.0) {
            // invalid detection
            return null;
        }

        double areaFraction = areaPercent / 100.0; // Convert area percent to fraction

        // Approximate vertical pixel height from area fraction (assumes roughly
        // square/circle object)
        double pPixels = Math.sqrt(areaFraction * IMAGE_WIDTH * IMAGE_HEIGHT);

        // Compute range using pinhole model
        double R = (FY * OBJECT_HEIGHT_M) / pPixels;

        // Convert angles to radians
        double theta = Math.toRadians(yawDeg); // yaw
        double phi = Math.toRadians(pitchDeg); // pitch

        // Convert spherical -> Cartesian (camera coordinates)
        double X = R * Math.cos(phi) * Math.cos(theta); // forward
        double Y = -R * Math.cos(phi) * Math.sin(theta); // right = negative
        double Z = R * Math.sin(phi); // up

        // Fix the errors
        X = X * xFix;
        Y = Y * yFix;
        Z = Z * zFix;

        return new Transform3d(new Translation3d(X, Y, Z), new Rotation3d());
    }

    /**
     * Get the type of object the camera detects.
     * 
     * @return Type enum representing the camera type.
     */
    public Type getCameraType() {
        return type;
    }

    /**
     * Get the current primary pose strategy used by the pose estimator.
     * 
     * @return PoseStrategy representing the current primary pose strategy, or null
     *         if not an AprilTag camera.
     */
    public PoseStrategy getPrimaryPoseStrategy() {
        if (!type.equals(Type.APRIL_TAG)) { // If the type is not for an AprilTag, return null
            return null;
        }

        return poseEstimator.getPrimaryStrategy();
    }

    /**
     * Change the primary pose strategy used by the pose estimator.
     * 
     * @param newStrategy - the new PoseStrategy to set as primary.
     */
    public void changePrimaryPoseStrategy(PoseStrategy newStrategy) {
        if (!type.equals(Type.APRIL_TAG)) { // If the type is not for an AprilTag, do nothing
            return;
        }

        poseEstimator.setPrimaryStrategy(newStrategy); // Change the primary pose strategy of the pose estimator
    }
}