package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.subsystem.SpikeSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class Vision extends SpikeSubsystem {
    private final VisionIO.VisionIOInputs frontInputs = new VisionIO.VisionIOInputs();
    private final VisionIOLimelight frontLimelight;

    public Vision(CommandSwerveDrivetrain drivetrain) {
        super("Vision");

        this.frontLimelight = new VisionIOLimelight(
                new LimelightConfig(
                        "limelight-front",
                        new Pose3d(
                                new Translation3d(
                                        0, 0, 0
                                ),
                                new Rotation3d(
                                        0, 0, 0
                                )
                        )
                ),
                drivetrain
        );
    }

    @Override
    protected Runnable setupDataRefresher() {
        return () -> {
            useDataRefresher(frontInputs, frontLimelight).run();
        };
    }
}