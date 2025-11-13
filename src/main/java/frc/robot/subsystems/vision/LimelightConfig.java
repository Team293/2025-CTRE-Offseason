package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;

public class LimelightConfig {
    private final String limelightName;
    private final Pose3d robotToCamera;
    private final Vector<N3> aprilTagVisionStdDevs;

    public LimelightConfig(String limelightName, Pose3d robotToCamera, Vector<N3> aprilTagVisionStdDevs) {
        this.limelightName = limelightName;
        this.robotToCamera = robotToCamera;
        this.aprilTagVisionStdDevs = aprilTagVisionStdDevs;
    }

    public LimelightConfig(String limelightName, Pose3d robotToCamera) {
        this(limelightName, robotToCamera, VecBuilder.fill(0.3, 0.3, 99999.0));
    }

    public String getLimelightName() {
        return limelightName;
    }

    public Pose3d getRobotToCamera() {
        return robotToCamera;
    }

    public Vector<N3> getAprilTagVisionStdDevs() {
        return aprilTagVisionStdDevs;
    }
}
