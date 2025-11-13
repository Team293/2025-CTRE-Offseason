package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.PositionVoltage;
import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO extends BaseIO<ElevatorIO.ElevatorIOInputs> {
    @AutoLog
    public static class ElevatorIOInputs extends BaseInputClass {
        public double positionValue = 0.0d;
        public double velocityValue = 0.0d;
    }

    void applyPosition(PositionVoltage request);
}
