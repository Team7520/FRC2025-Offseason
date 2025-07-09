// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

//import java.util.List;

import org.photonvision.PhotonCamera;
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
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
//import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.Constants;

//import frc.team7520.robot.Constants;
import java.io.IOException;

/** Add your docs here. */
public class AprilTagSystem {

    private PhotonCamera camera1;
    private PhotonCamera camera2;
    private PhotonCamera camera3;
    private PhotonCamera camera4;
    private boolean isCamera1Open = false;
    private boolean isCamera2Open = false;
    private boolean isCamera3Open = false;
    private boolean isCamera4Open = false;
    private final PipeLineType TYPE = PipeLineType.APRIL_TAG;
    private boolean isOpen = false;
    private boolean facingTarget = false;
    private String cameraName1;
    private String cameraName2;
    private String cameraName3;
    private String cameraName4;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private List<AprilTag> apriltags;
    private boolean aprilTagLayoutLoaded = false;
    private final double MAX_RANGE = 2; //In meters, anything beyond 2 meters should not be used
    
    private Pose2d robotPose;
    private AprilTag closestTag;

    public enum PipeLineType {
        APRIL_TAG,
        COLORED_SHAPE,
        REFLECTIVE
    }

    public class PhotonVisionData {
        public boolean is_vaild;
        // 2D Mode
        public double yaw;
        public double pitch;
        public double area;
        public double skew;
        public double x_dist_2d;
        public double y_dist_2d;
        // 3D Mode
        public int april_tag_id;
        public double x_distance;
        public double y_distance;
        public double z_distance;
        public double x_rotate;
        public double y_rotate;
        public double z_rotate;
        public double angle_rotate;
        public double ambiguity;
    }

    public AprilTagSystem(String cameraName1, String cameraName2, String cameraName3, String cameraName4) {
        periodic(robotPose);
        this.cameraName1 = cameraName1; 
        this.cameraName2 = cameraName2;
        this.cameraName3 = cameraName3;
        this.cameraName4 = cameraName4;
               
        camera1 = new PhotonCamera(cameraName1);
        camera2 = new PhotonCamera(cameraName2);
        camera3 = new PhotonCamera(cameraName3);
        camera4 = new PhotonCamera(cameraName4);
    }

    public void periodic(Pose2d robotPose) {
        this.robotPose = robotPose;
        if(camera1.isConnected() && camera2.isConnected() && camera3.isConnected() && camera4.isConnected()) {
            isOpen = true;
        }
        SmartDashboard.putBoolean("PHOTONVISION ALL CONNECTED?", isOpen);
        if(!camera1.isConnected()) {
            System.out.printf("Failed to open camera 1: %s \n", cameraName1);
            isCamera1Open = false;
            SmartDashboard.putBoolean("CAMERA 1 OPEN?", isCamera1Open);
        } else {
            isCamera1Open = true;
            SmartDashboard.putBoolean("CAMERA 1 OPEN?", isCamera1Open);
        }

        if(!camera2.isConnected()) {
            System.out.printf("Failed to open camera 2: %s \n", cameraName2);
            isCamera2Open = false;
            SmartDashboard.putBoolean("CAMERA 2 OPEN?", isCamera2Open);
        } else {
            isCamera2Open = true;
            SmartDashboard.putBoolean("CAMERA 2 OPEN?", isCamera2Open);
        }

        if(!camera3.isConnected()) {
            System.out.printf("Failed to open camera 3: %s \n", cameraName3);
            isCamera3Open = false;
            SmartDashboard.putBoolean("CAMERA 3 OPEN?", isCamera3Open);
        } else {
            isCamera3Open = true;
            SmartDashboard.putBoolean("CAMERA 3 OPEN?", isCamera3Open);
        }

        if(!camera4.isConnected()) {
            System.out.printf("Failed to open camera 4: %s \n", cameraName4);
            isCamera4Open = false;
            SmartDashboard.putBoolean("CAMERA 4 OPEN?", isCamera4Open);
        } else {
            isCamera4Open = true;
            SmartDashboard.putBoolean("CAMERA 4 OPEN?", isCamera4Open);
        }
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

    /**
     * Returns the ambiugity of the given camera, used to determine which has the most accurate data
     * @return a double representing the ambiguity of the camera
     */
    public double getAmbiguity(int camera) {
        PhotonPipelineResult result = null;
        if(camera == 1) {
            result = camera1.getLatestResult();
        } else if(camera == 2) {
            result = camera2.getLatestResult();
        } else if(camera == 3) {
            result = camera3.getLatestResult();
        } else if(camera == 4) {
            result = camera4.getLatestResult();
        }

        if(result == null || !result.hasTargets()) {
            return -1; // Handle the case where no camera matches or no targets are found
        }
        
        PhotonTrackedTarget target = result.getBestTarget();
        return target.getPoseAmbiguity();
    }

    /**
     * Returns the capture time, used for pose estimator
     * @return capture time in milliseconds
     */
    public double getCaptureTime(int camera) {
        PhotonPipelineResult result = null;
        if(camera == 1) {
            result = camera1.getLatestResult();
        } else if(camera == 2) {
            result = camera2.getLatestResult();
        } else if(camera == 3) {
            result = camera3.getLatestResult();
        } else if(camera == 4) {
            result = camera4.getLatestResult();
        }

        if(result == null || !result.hasTargets()) {
            return -1; // Handle the case where no camera matches or no targets are found
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
        if(camera == 1) {
            result = camera1.getLatestResult();
        } else if(camera == 2) {
            result = camera2.getLatestResult();
        } else if(camera == 3) {
            result = camera3.getLatestResult();
        } else if(camera == 4) {
            result = camera4.getLatestResult();
        }

        if(result == null) {
            return null; // Handle the case where no camera matches
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
        double CAMERA_POS_FOR_ROBOT_X = -0.254; // Meters
        double CAMERA_POS_FOR_ROBOT_Y = 0;
        double CAMERA_POS_FOR_ROBOT_Z = 0.6858;
        double CAMERA_POS_FOR_ROBOT_ROLL = 0;
        double CAMERA_POS_FOR_ROBOT_PITCH = -Math.toRadians(35); // Radians
        double CAMERA_POS_FOR_ROBOT_YAW = Math.PI;

        Transform3d robotToCamera = new Transform3d(CAMERA_POS_FOR_ROBOT_X, 
                                                    CAMERA_POS_FOR_ROBOT_Y, 
                                                    CAMERA_POS_FOR_ROBOT_Z, 
                                                    new Rotation3d(CAMERA_POS_FOR_ROBOT_ROLL,
                                                    CAMERA_POS_FOR_ROBOT_PITCH,
                                                    CAMERA_POS_FOR_ROBOT_YAW));
        
        if(result.hasTargets()) {
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
        return null;
    }
}