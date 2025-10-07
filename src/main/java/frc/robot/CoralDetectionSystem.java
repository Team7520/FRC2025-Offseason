package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.InterpolatingYawPitch2D;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class CoralDetectionSystem extends SubsystemBase {
    public Pose2d coralPose;
    public Translation2d robotCoralTranslation;
    private final PhotonCamera camera;
    private Pose2d robotPose;
    private final SwerveSubsystem swerveSubsystem;

    // Interpolating maps for estimating distances
    private final InterpolatingDoubleTreeMap areaToDistanceMap = new InterpolatingDoubleTreeMap();
    InterpolatingYawPitch2D interpolator = new InterpolatingYawPitch2D();

    // Confidence threshold for detection
    private static final double CONFIDENCE_THRESHOLD = 0.6; /* TODO: set confidence threshold, e.g. 0.6 */

    // The class label you want to detect (may not be available if using standard PhotonVision)
    private static final String TARGET_CLASS = "coral"; // TODO: change if needed

    StructPublisher<Pose2d> coralPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("coralPose", Pose2d.struct).publish();

    public CoralDetectionSystem(String cameraName, SwerveSubsystem swerveSubsystem) {
        camera = new PhotonCamera(cameraName);
        this.swerveSubsystem = swerveSubsystem;

        interpolator.put(1.2, -1, 0, -0.5); // yaw, pitch,,
        interpolator.put(5, 11.29, 0, -1);
        interpolator.put(6.89, 18, 0, -1.5);

        interpolator.put(14, -1.9, 0.25, -0.5);
        interpolator.put(14.3, 6, 0.25, -0.75);
        interpolator.put(14.9, 10.8, 0.25, -1);
        interpolator.put(14.8, 15.3, 0.25, -1.25);

        interpolator.put(24.64, -2.05, 0.5, -0.5);
        interpolator.put(24.31, 11.02, 0.5, -1);
        interpolator.put(23.64, 17.91, 0.5, -1.5);

        interpolator.put(-7.5, 6.14, -0.25, -0.75);
        interpolator.put(-4.6, 10.96, -0.25, -1);
        interpolator.put(-2.8, 14.9, -0.25, -1.25);

        interpolator.put(-21.54, -0.93, -0.5, -0.5);
        interpolator.put(-12.15, 10.37, -0.5, -1);
        interpolator.put(-6.8, 17.19, -0.5, -1.5);

        interpolator.put(-15, 15.2, 0.25, -1.25);
        interpolator.put(-1.90, 14.75, -0.25, -1.25);
    }

    /** @return true if a valid gamepiece (coral) is detected above confidence threshold */
    public boolean isGamepieceDetected() {
        PhotonTrackedTarget target = getBestTarget();
        if (target == null) return false;
        if (target.getDetectedObjectConfidence() < CONFIDENCE_THRESHOLD) return false;
        return true;
    }

    /** @return the confidence of the current detection */
    public double getDetectionConfidence() {
        PhotonTrackedTarget target = getBestTarget();
        return target != null ? target.getDetectedObjectConfidence() : 0.0;
    }

    public int getID() {
        PhotonTrackedTarget target = getBestTarget();
        return target != null ? target.getFiducialId() : 0;
    }

    /** @return pitch value from detection (degrees) */
    public double getPitch() {
        PhotonTrackedTarget target = getBestTarget();
        return target != null ? target.getPitch() : 0.0;
    }

    /** @return yaw value from detection (degrees) */
    public double getYaw() {
        PhotonTrackedTarget target = getBestTarget();
        return target != null ? target.getYaw() : 0.0;
    }

    /** @return area of detected object (normalized 0-100) */
    public double getArea() {
        PhotonTrackedTarget target = getBestTarget();
        return target != null ? target.getArea() : 0.0;
    }

    /** @return skew of detected object (degrees, if available) */
    public double getSkew() {
        PhotonTrackedTarget target = getBestTarget();
        return target != null ? target.getSkew() : 0.0;
    }

    /**
     * @return estimated straight-line distance (Y) to the gamepiece based on area
     */
    public double getEstimatedDistance() {
        double area = getArea();
        if (area <= 0) return -1;
        Double distance = areaToDistanceMap.get(area);
        return distance != null ? distance : -1;
    }

    /**
     * @return estimated X offset to the gamepiece based on yaw
     */
    public double getEstimatedX() {
        double yaw = getYaw();
        double pitch = getPitch();
        double[] xy = interpolator.getInterpolated(yaw, pitch);
        if (xy == null) return -1;
        return xy[1];
    }

    /**
     * @return estimated Y offset to the gamepiece based on pitch or area
     */
    public double getEstimatedY() {
        double yaw = getYaw();
        double pitch = getPitch();
        double[] xy = interpolator.getInterpolated(yaw, pitch);
        if (xy == null) return -1;
        return xy[0];
    }

    /** @return the best detected target, or null if none */
    private PhotonTrackedTarget getBestTarget() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) return null;
        return result.getBestTarget();
    }

    /** @return estimated Pose2d of the gamepiece relative to the robot, or null if not detected */
    public Translation2d getRobotCoralTranslation() {
        if (!isGamepieceDetected()) return null;
        double x = getEstimatedX();
        double y = getEstimatedY();
        if (x == -1 || y == -1) return null;
        robotCoralTranslation = new Translation2d(x, y);
        return robotCoralTranslation;
    }

    /**
     * @param robotPose The robot's field pose.
     * @return The field-relative pose of the coral, or null if not detected.
     */
    public Pose2d getCoralPose(Pose2d robotPose) {
        Translation2d translation = getRobotCoralTranslation();
        if (translation != null) {
            return robotPose.plus(new Transform2d(translation, new Rotation2d()));
        } else {
            return null;
        }
    }

    public void periodic() {
        robotPose = swerveSubsystem.getPose();
        coralPose = getCoralPose(robotPose);
        SmartDashboard.putString("Coral Pose", coralPose != null ? coralPose.toString() : "None");
        if (coralPose != null) {
            coralPosePublisher.set(coralPose);
        }
        SmartDashboard.putBoolean("VisionCoralDetected", isGamepieceDetected());
        SmartDashboard.putNumber("Coral Yaw", getYaw());
        SmartDashboard.putNumber("Coral Pitch", getPitch());
        SmartDashboard.putNumber("Coral Y", getEstimatedY());
        SmartDashboard.putNumber("Coral X", getEstimatedX());
    }

    // TODO: Add methods to update/populate interpolating maps from calibration data, if needed
}