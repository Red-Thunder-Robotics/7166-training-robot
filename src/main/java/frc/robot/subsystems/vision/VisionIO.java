package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        boolean megaTagSuccess;
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
