package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        boolean megaTagSuccess;
        boolean ntConnected;
    }

    @AutoLog
    public static class AprilTagVisionIOInputs {
        double[] timestamps = new double[] {};
        double[][] frames = new double[][] {};
        long fps = 0;
    }
    @AutoLog
    public static class ObjDetectVisionIOInputs {
        double[] timestamps = new double[] {};
        double[][] frames = new double[][] {};
        long fps = 0;
    }

    public default void updateInputs(VisionIOInputs inputs) {}
    public default void updateInputs(VisionIOInputs inputs, AprilTagVisionIOInputs aprilTagInputs, ObjDetectVisionIOInputs objDetectInputs) {}
}
