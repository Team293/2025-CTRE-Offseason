package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.subsystem.IORefresher;

public class ClimberIOTalonFX implements ClimberIO, IORefresher {
    private final TalonFX climberMotor;
    private final StatusSignal<AngularVelocity> currentVelocity;
    private final StatusSignal<Voltage> supplyVoltage;

    private static VelocityVoltage command = new VelocityVoltage(0.0d).withSlot(0);

    public ClimberIOTalonFX(int canId) {
        this.climberMotor = new TalonFX(canId);
        var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = 1.0;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = 0.3; 
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kV = 0.0; 
        config.Slot0.kS = 0.14; 

        climberMotor.getConfigurator().apply(config);

        currentVelocity = climberMotor.getVelocity();
        supplyVoltage = climberMotor.getSupplyVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                currentVelocity,
                supplyVoltage
        );

        climberMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.currentVelocity = currentVelocity.getValueAsDouble();
        inputs.supplyVoltage = supplyVoltage.getValueAsDouble();
    }

    @Override
    public void setSpeed(double speed) {
        command.withVelocity(speed).withSlot(0);
        climberMotor.setControl(command);
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
                currentVelocity,
                supplyVoltage
        );
    }
}
