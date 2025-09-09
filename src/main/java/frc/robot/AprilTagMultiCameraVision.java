package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagMultiCameraVision {
    private final List<PhotonCamera> cameras;
    private final List<PhotonPoseEstimator> estimators;
    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;

    // Standard deviations (tune these for your robot)
    private final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    private final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 0.5);

    /**
     * @param cameras List of all PhotonCamera objects
     * @param estimators List of PhotonPoseEstimator objects, one per camera (with MULTI_TAG_PNP_ON_COPROCESSOR)
     * @param estConsumer Lambda to accept pose, timestamp, and stdDevs for fusion
     */
    public AprilTagMultiCameraVision(
        List<PhotonCamera> cameras,
        List<PhotonPoseEstimator> estimators,
        EstimateConsumer estConsumer
    ) {
        this.cameras = cameras;
        this.estimators = estimators;
        this.estConsumer = estConsumer;
    }

    /** Call this in your periodic loop. Pass current odometry pose. */
    public void periodic(Pose2d referencePose) {
        for (int camIdx = 0; camIdx < cameras.size(); camIdx++) {
            PhotonCamera camera = cameras.get(camIdx);
            PhotonPoseEstimator estimator = estimators.get(camIdx);

            for (var result : camera.getAllUnreadResults()) {
                estimator.setReferencePose(referencePose);

                Optional<EstimatedRobotPose> visionEst = estimator.update(result);
                updateEstimationStdDevs(estimator, visionEst, result.getTargets());

                visionEst.ifPresent(est -> {
                    var estStdDevs = getEstimationStdDevs();
                    estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });
            }
        }
    }

    /**
     * Update measurement std devs using a heuristic: more tags = less uncertainty, farther = more uncertainty.
     * @param estimator The pose estimator for the camera
     * @param estimatedPose Optional vision pose
     * @param targets List of visible targets in frame
     */
    private void updateEstimationStdDevs(
        PhotonPoseEstimator estimator,
        Optional<EstimatedRobotPose> estimatedPose,
        List<PhotonTrackedTarget> targets
    ) {
        if (estimatedPose.isEmpty()) {
            curStdDevs = kSingleTagStdDevs;
        } else {
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            for (var tgt : targets) {
                var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                    tagPose.get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                curStdDevs = kSingleTagStdDevs;
            } else {
                avgDist /= numTags;
                // Decrease std devs if multiple tags
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs if far away or only one tag
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations for the estimated pose.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    /** Lambda for pose estimator fusion: (pose, timestamp, stdDevs) */
    @FunctionalInterface
    public static interface EstimateConsumer {
        void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}