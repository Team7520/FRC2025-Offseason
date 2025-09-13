// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

//import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
//import org.photonvision.targeting.TargetCorner;

import com.fasterxml.jackson.databind.util.JSONPObject;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;

/** Add your docs here. */
public class AprilTagSystem {
    
    public static class CameraInfo {
        public final String name;
        public final PhotonCamera camera;
        public boolean isOpen;
        public final Transform3d robotToCamera;

        public CameraInfo(String name, PhotonCamera camera, boolean isOpen, Transform3d robotToCamera) {
            this.name = name;
            this.camera = camera;
            this.isOpen = isOpen;
            this.robotToCamera = robotToCamera;
        }
    }

    private final List<CameraInfo> cameraList = new ArrayList<>();
    
    private final PipeLineType TYPE = PipeLineType.APRIL_TAG;
    private boolean allOpen = false;
    private boolean facingTarget = false;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private List<AprilTag> apriltags;
    public boolean aprilTagLayoutLoaded = false;
    private final double MAX_RANGE = 20; //In meters, anything beyond 2 meters should not be used
    
    private Pose2d robotPose;
    private AprilTag closestTag;
    private List<PhotonPoseEstimator> estimators = null;

    public enum PipeLineType {
        APRIL_TAG,
        COLORED_SHAPE,
        REFLECTIVE
    }

    public class PhotonVisionData {
    }

    public AprilTagSystem() {
        // Initialize the cameras
        cameraList.add(new CameraInfo(
            "FrontRightCam",
            new PhotonCamera("FrontRightCam"),
            false,
            new Transform3d(
                0.282121, // X forward
                -0.189152, // Y right
                0.206491,
                new Rotation3d(
                    0.0,
                    Math.toRadians(10), // pitched up
                    0.0 // facing forward
                )
            )
        ));

        // cameraList.add(new CameraInfo(
        //     "FrontLeftCam",
        //     new PhotonCamera("FrontLeftCam"),
        //     false,
        //     new Transform3d(
        //         0.282121, // X forward
        //         0.189152, // Y left
        //         0.206491,
        //         new Rotation3d(
        //             0.0,
        //             Math.toRadians(10),
        //             0.0
        //         )
        //     )
        // ));

        cameraList.add(new CameraInfo(
            "BackRightCam",
            new PhotonCamera("BackRightCam"),
            false,
            new Transform3d(
                -0.282121, // X backward
                -0.189152, // Y right
                0.206491,
                new Rotation3d(
                    0.0,
                    Math.toRadians(10),
                    Math.PI // facing backward
                )
            )
        ));

        cameraList.add(new CameraInfo(
            "BackLeftCam",
            new PhotonCamera("BackLeftCam"),
            false,
            new Transform3d(
                -0.282121, // X backward
                0.189152,  // Y left
                0.206491,
                new Rotation3d(
                    0.0,
                    Math.toRadians(10),
                    Math.PI
                )
            )
        ));
        periodic(robotPose);
        
    }

    public void periodic(Pose2d robotPose) {
        this.robotPose = robotPose;
        allOpen = true;
        for(int i = 0; i < cameraList.size(); i++) {
            if (cameraList.get(i).camera.isConnected()) {
                cameraList.get(i).isOpen = true;
            } else {
                cameraList.get(i).isOpen = false;
                allOpen = false;
                System.out.printf("Failed to open camera %d: %s \n", i + 1, cameraList.get(i).name);
            }
            SmartDashboard.putBoolean(cameraList.get(i).name + " OPEN?", cameraList.get(i).isOpen);
        }
    }


    public List<PhotonCamera> getCameras() {
        return cameraList.stream()
            .map(info -> info.camera)
            .collect(Collectors.toList());
    }
    public List<PhotonPoseEstimator> getEstimators() {
        // Ensure layout is loaded first
        if (!aprilTagLayoutLoaded) {
            initiateAprilTagLayout();
        }
        // Only build estimators once
        if (estimators == null && aprilTagLayoutLoaded) {
            estimators = cameraList.stream()
                .map(info -> new PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    info.robotToCamera
                ))
                .collect(Collectors.toList());
        }
        return estimators == null ? List.of() : estimators;
    }

    public int getCameraCount() {
        return cameraList.size();
    }
    /** 
     * Loads field layout of april tags. The aprilTagFieldLayout field is initiated here because the layout takes a while to load - often crashes
     * if called too soon (such as within class constructor). 
     * @return a boolean indicating whether the layout was loaded
     */
    public boolean initiateAprilTagLayout() {
        for (int i = 0; i < 5 && !aprilTagLayoutLoaded; i++) {
            try {            
                aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);
                apriltags = aprilTagFieldLayout.getTags();
                aprilTagLayoutLoaded = true;
            } catch (IOException e) {
                e.printStackTrace();
                System.out.println("Load April Tag Layout Error");
            }
        }
        return aprilTagLayoutLoaded;
    }

    public AprilTagFieldLayout getAprilTagLayout() {
        return aprilTagFieldLayout;
    }

    /**
     * Returns the ambiguity of the given camera, used to determine which has the most accurate data
     * @return a double representing the ambiguity of the camera
     */
    public double getClosest(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= cameraList.size()) {
            return -1; // Handle invalid camera index
        }

        CameraInfo cameraInfo = cameraList.get(cameraIndex);
        PhotonPipelineResult result = cameraInfo.camera.getLatestResult();

        if (result == null || !result.hasTargets()) {
            return -1; // Handle the case where no targets are found
        }

        PhotonTrackedTarget target = result.getBestTarget();
        Transform3d targets = target.getBestCameraToTarget();
        double xValue = targets.getX();
        double yValue = targets.getY();
        double distance = Math.sqrt(Math.pow(xValue, 2) + Math.pow(yValue, 2));
        return distance;
    }

    /**
     * Returns the capture time, used for pose estimator
     * @return capture time in milliseconds
     */
    public double getCaptureTime(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= cameraList.size()) {
            return -1; // Handle invalid camera index
        }

        CameraInfo cameraInfo = cameraList.get(cameraIndex);
        PhotonPipelineResult result = cameraInfo.camera.getLatestResult();

        if (result == null || !result.hasTargets()) {
            return -1; // Handle the case where no targets are found
        }

        return result.getTimestampSeconds() * 1000; // Convert seconds to milliseconds
    }

    /**
     * Estimates the current robot position based on the april tag it sees. April tags farther than {@link #MAX_RANGE}
     * are not considered.
     * @return a Pose2d
     */
    public Pose2d getCurrentRobotFieldPose(int camera) {
        PhotonPipelineResult result = null;
        if (camera >= 0 && camera < cameraList.size()) {
            result = cameraList.get(camera).camera.getLatestResult();
        }

        if (result == null || !result.hasTargets()) {
            return null;
        }
        /* 
         * To avoid confusion regarding whether rotation is applied first or translation, and whether the rotation/translational
         * axes are transformed along with the object, we'll assume translation is considered first before rotation, where each
         * component of the transformation is considered in the order as it is labeled as a parameter.
         * 
         * According to WPILIB Documentation, the related object/class, Transform2d, consists of a translation and a rotation.
         * In Transform2d, the rotation is applied TO THE TRANSLATION, then the rotation is applied to the object.
         * This is mathematically equivalent to applying an unrotated translation, then applying the rotation to the object.
         * Both seqences transform the object to the same destination in space.
         * 
         * For simplicity reasons, we should base the camera's transformation relative to the robot center, and then simply inverse
         * the transformtation - instead of having robot relative to camera.
         * 
         * -Robin
         */
        Transform3d robotToCamera = cameraList.get(camera).robotToCamera;
        
        PhotonTrackedTarget target = result.getBestTarget();
        if (target.getBestCameraToTarget().getX() > MAX_RANGE) {
            return null;
        }
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
            target.getBestCameraToTarget(), 
            aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), 
            robotToCamera.inverse());
        //SmartDashboard.putNumber("Tag X", target.getBestCameraToTarget().getX());
        //SmartDashboard.putNumber("Tag Y", target.getBestCameraToTarget().getY());
        return robotPose.toPose2d();            
        
    }

        /**
     * Finds the closest visible AprilTag from all available cameras.
     * Returns the field-relative pose of that tag.
     *
     * @return Pose2d of the closest visible tag, or null if none seen
     */
    public Pose2d getClosestTagPose() {
        if (!aprilTagLayoutLoaded) {
            initiateAprilTagLayout();
          }
        if (!aprilTagLayoutLoaded) {
            System.out.println("!aprilTagLayoutLoaded");
            return null;
        }

        Pose2d closestTagPose = null;
        double closestDistance = Double.MAX_VALUE;

        for (CameraInfo cam : cameraList) {
            PhotonPipelineResult result = cam.camera.getLatestResult();
            if (result == null || !result.hasTargets()) {
                continue;
            }

            PhotonTrackedTarget target = result.getBestTarget();
            if (!aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
                continue;
            }

            double distance = target.getBestCameraToTarget().getTranslation().getNorm();
            if (distance > MAX_RANGE) {
                continue;
            }

            Pose3d tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();
            if (distance < closestDistance) {
                closestDistance = distance;
                closestTagPose = tagPose.toPose2d();
                this.closestTag = new AprilTag(target.getFiducialId(), tagPose);
            }
        }

        return closestTagPose;
    }

        /**
     * Returns a new Pose2d offset from the given tag pose.
     * 
     * @param tagPose       The Pose2d of the AprilTag
     * @param forwardMeters Distance in front of the tag (+ forward, - behind)
     * @param lateralMeters Distance to the right of the tag (+ right, - left)
     * @return Offset Pose2d
     */
    public Pose2d getOffsetPose(Pose2d tagPose, double forwardMeters, double lateralMeters) {
        // Construct a transform in the tag's frame: forward + right
        Transform2d offset = new Transform2d(
            new Translation2d(forwardMeters, lateralMeters),
            Rotation2d.fromDegrees(180) // Keep original tag orientation
        );

        // Apply the transform relative to the tag pose
        return tagPose.plus(offset);
    }

    private static final Set<Integer> reefscapeTagIDs = Set.of(
    6,7,8,9,10,11,17,18,19,20,21,22
    );

    public Pose2d getNearestTagPose(Pose2d robotPose) {
        double minDistance = Double.MAX_VALUE;
        Pose2d closestPose = null;

        if (!aprilTagLayoutLoaded){
            initiateAprilTagLayout();
        }
        for (AprilTag tag : aprilTagFieldLayout.getTags()) {
            if (!reefscapeTagIDs.contains(tag.ID)) continue; // filter only reef tags

            Pose2d tagPose = new Pose2d(
                tag.pose.getX(),
                tag.pose.getY(),
                tag.pose.getRotation().toRotation2d()
            );

            double distance = robotPose.getTranslation().getDistance(tagPose.getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                closestPose = tagPose;
            }
        }

        return closestPose;
    }

    public Pose2d getAlignPose(Pose2d robotPose, Pose2d tagPose, boolean faceFront) {
        Translation2d toTag = tagPose.getTranslation().minus(robotPose.getTranslation());
        double angleToTag = Math.atan2(toTag.getY(), toTag.getX());
        Rotation2d desiredRotation = Rotation2d.fromRadians(angleToTag);
    
        if (!faceFront) {
            desiredRotation = desiredRotation.rotateBy(Rotation2d.fromDegrees(180));
        }
    
        return new Pose2d(tagPose.getTranslation(), desiredRotation);
    }
    public Pose2d getOptimalAlignPose(Pose2d robotPose, Pose2d poseA, Pose2d poseB) {
        
        double costA = robotPose.getTranslation().getDistance(poseA.getTranslation())
                      + Math.abs(robotPose.getRotation().minus(poseA.getRotation()).getRadians());
        double costB = robotPose.getTranslation().getDistance(poseB.getTranslation())
                      + Math.abs(robotPose.getRotation().minus(poseB.getRotation()).getRadians());
        return (costA <= costB) ? poseA : poseB;
    }
}