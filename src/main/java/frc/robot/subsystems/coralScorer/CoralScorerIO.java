package frc.robot.subsystems.coralScorer;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;

public interface CoralScorerIO extends BaseIO<CoralScorerIO.CoralScorerIOInputs> {
    @AutoLog
    public static class CoralScorerIOInputs extends BaseInputClass {
        public double speed = 0.0d;
        public double current = 0.0d;
    }

    void setSpeed(double speed);
}
