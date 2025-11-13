package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class Constants {
    public static final Mode ROBOT_MODE = Mode.REAL;
    public static final boolean ENABLE_SYSID_BINDINGS = false;
    public static final Vector<N3> VISION_STD_DEVS = VecBuilder.fill(0.3, 0.3, 99999.0);


    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }
}
