package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.LimelightHelpers;
import frc.lib.subsystem.IORefresher;
import frc.robot.Constants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class VisionIOLimelight implements VisionIO, IORefresher {
    private Pose2d latestEstimate = new Pose2d();
    private final LimelightConfig limelightConfig;
    private final CommandSwerveDrivetrain drivetrain;

    public VisionIOLimelight(LimelightConfig limelightConfig, CommandSwerveDrivetrain drivetrain) {
        this.limelightConfig = limelightConfig;
        this.drivetrain = drivetrain;

        var robotToCamera = limelightConfig.getRobotToCamera();

        LimelightHelpers.setCameraPose_RobotSpace(
                limelightConfig.getLimelightName(),
                robotToCamera.getX(),
                robotToCamera.getY(),
                robotToCamera.getZ(),
                Math.toDegrees(robotToCamera.getRotation().getX()),
                Math.toDegrees(robotToCamera.getRotation().getY()),
                Math.toDegrees(robotToCamera.getRotation().getZ())
        );
    }

    @Override
    public void refreshData() {
        // update the robot orientation (need for mt2)
        Rotation2d theta = drivetrain.getState().Pose.getRotation();
        LimelightHelpers.SetRobotOrientation(
                limelightConfig.getLimelightName(),
                theta.getDegrees(),
                0,
                0,
                0,
                0,
                0
        );

        setLatestEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightConfig.getLimelightName()), 1);
    }

    @Override
    public void setLatestEstimate(LimelightHelpers.PoseEstimate pose, int minTags) {
        if (pose.tagCount >= minTags) {
            latestEstimate = pose.pose;
            drivetrain.addVisionMeasurement(
                    latestEstimate,
                    pose.timestampSeconds,
                    Constants.VISION_STD_DEVS.times(pose.avgTagDist)
            );
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.pose = latestEstimate;
    }
}
