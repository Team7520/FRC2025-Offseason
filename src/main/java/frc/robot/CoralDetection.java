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
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


import java.io.IOException;

/** Add your docs here. */
public class CoralDetection {    
    public final PhotonCamera camera;
    private PhotonPipelineResult result = new PhotonPipelineResult();
    private final SwerveSubsystem drivebase;
    private double xCameraOffset = Units.inchesToMeters(-6.911293); // meters
    private double yCameraOffset = Units.inchesToMeters(-10.054); // meters
    private double cameraAngleOffset = 15; // degrees

    public CoralDetection(SwerveSubsystem drivebase) {
        camera = new PhotonCamera("Arducam_OV9782_USB_Camera (1)");
        this.drivebase = drivebase;
    }

    Transform2d cameraToRobot = new Transform2d(
        new Translation2d(xCameraOffset, yCameraOffset),
        new Rotation2d(Units.degreesToRadians(cameraAngleOffset))
    );

    public void periodic(Pose2d robotPose) {
        result = camera.getLatestResult();
        if(hasTarget()) {
            PhotonTrackedTarget target = result.getBestTarget();
            if(target.getDetectedObjectClassID() == 0) {
                double yaw = target.getYaw();
                double pitch = target.getPitch();
                double area = target.getArea();
                double skew = target.getSkew();
                SmartDashboard.putNumber("Coral Yaw", yaw);
                SmartDashboard.putNumber("Coral Pitch", pitch);
                SmartDashboard.putNumber("Coral Area", area);
                SmartDashboard.putNumber("Coral Skew", skew);
                SmartDashboard.putNumber("Confidence", target.getDetectedObjectConfidence());
            } else {
                SmartDashboard.putNumber("Coral Yaw", 0);
                SmartDashboard.putNumber("Coral Pitch", 0);
                SmartDashboard.putNumber("Coral Area", 0);
                SmartDashboard.putNumber("Coral Skew", 0);
                SmartDashboard.putNumber("Confidence", 0);
            }
        }
    }

    public boolean hasTarget() {
        return result.hasTargets();
    }

    public Pose2d getCoralPos(Pose2d robotPose) {
        if (!hasTarget()) return new Pose2d();

        PhotonTrackedTarget target = result.getBestTarget();
        if (target.getDetectedObjectClassID() != 0 || target.getDetectedObjectConfidence() <= 0.7)
            return new Pose2d();

        // Camera parameters
        double cameraHeight = Units.inchesToMeters(28.137);
        double targetHeight = 0.1;
        double cameraPitch = Units.degreesToRadians(-40);
        double pitchRad = Units.degreesToRadians(target.getPitch());

        double distance = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetHeight, cameraPitch, pitchRad);

        Pose2d cameraPose = robotPose.transformBy(cameraToRobot);
        Translation2d offset = new Translation2d(distance, 0).rotateBy(cameraPose.getRotation());

        Pose2d coralPose = new Pose2d(
            cameraPose.getX() + offset.getX(),
            cameraPose.getY() + offset.getY(),
            cameraPose.getRotation() // or a rotation toward coral if desired
        );

        return coralPose;
    }


    public Rotation2d getYawError() {
        if(hasTarget()) {
            PhotonTrackedTarget target = result.getBestTarget();
            if(target.getDetectedObjectClassID() == 0 && target.getDetectedObjectConfidence() > 0.7) {
                return Rotation2d.fromDegrees(target.getYaw());
            } else {
                return new Rotation2d(0);
            }
        } else {
            return Rotation2d.fromDegrees(0);
        }
    }
}