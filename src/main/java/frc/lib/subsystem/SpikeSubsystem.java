package frc.lib.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.DataUtils;
import frc.lib.LoggedTracer;

public abstract class SpikeSubsystem extends SubsystemBase {
    private final Runnable dataRefreshTask;

    public SpikeSubsystem(String name) {
        super(name);
        this.dataRefreshTask = setupDataRefresher();
    }

    protected abstract Runnable setupDataRefresher();

    protected <I extends BaseInputClass, T extends BaseIO<I> & IORefresher> Runnable useAsyncDataRefresher(
            I inputs, T baseIO
    ) {
        DataUtils.createDataLogger(inputs, baseIO);
        return () -> {};
    }

    protected <I extends BaseInputClass, T extends BaseIO<I>> Runnable useDataRefresher(
            I inputs, T baseIO
    ) {
        return () -> baseIO.updateInputs(inputs);
    }

    public void onPeriodic() {}

    @Override
    public final void periodic() {
        dataRefreshTask.run();

        onPeriodic();
        LoggedTracer.record(getName());
    }
}