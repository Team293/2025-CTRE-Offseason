package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;

public interface ClimberIO extends BaseIO<ClimberIO.ClimberIOInputs> {
    @AutoLog
    public static class ClimberIOInputs extends BaseInputClass {
        public double currentVelocity = 0.0d;
        public double supplyVoltage = 0.0d;
    }

    void setSpeed(double speed);
}
