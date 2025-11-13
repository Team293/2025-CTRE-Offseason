package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.LimelightHelpers;
import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO extends BaseIO<VisionIO.VisionIOInputs> {
    @AutoLog
    public static class VisionIOInputs extends BaseInputClass {
        public Pose2d pose = new Pose2d();
    }

    void setLatestEstimate(LimelightHelpers.PoseEstimate pose, int minTags);
}
