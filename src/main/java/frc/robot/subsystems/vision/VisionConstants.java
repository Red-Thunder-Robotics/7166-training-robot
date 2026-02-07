package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Constants;

public final class VisionConstants {
    public static final String limelightFrontName = "limelight-front";
    public static final String limelightBackName = "limelight-back";

    private static final boolean forceEnableInstanceLogging = false;
    public static final boolean enableInstanceLogging =
        forceEnableInstanceLogging || Constants.CURRENT_MODE == Constants.Mode.REPLAY;

    public static final double ambiguityThreshold = 0.4d;
    public static final double targetLogTimeSecs = 0.1d;
    public static final double fieldBorderMargin = 0.5d;
    public static final double xyStdDevCoefficient = 0.01d;
    public static final double thetaStdDevCoefficient = 0.03d;
    public static final double demoTagPosePersistenceSecs = 0.5d;
    public static final double fuelDetectConfidenceThreshold = 0.35d;
    public static final double timestampOffset = 0d;

    public record CameraConfig(
        Supplier<Pose3d> pose,
        String id,
        int width,
        int height,
        int autoExposure,
        int exposure,
        double gain,
        double denoise,
        double stdDevFactor) {}

    public static CameraConfig[] cameras = new CameraConfig[] {
        // new CameraConfig( // logic 2
        //     () -> new Pose3d(
        //         -0.2794,
        //         0.2286,
        //         0.21113,
        //         new Rotation3d(
        //             Degrees.of(0d),
        //             // Units.degreesToRadians(-25d + pitchAdjustments[0].get()),
        //             Degrees.of(30d),
        //             // Units.degreesToRadians(-20d))),
        //             Degrees.of(180d))),
        //     // "0x1120:0x46D:0x825",
        //     "0x3120:0x46D:0x825",
        //     1280,
        //     720,
        //     0,
        //     200,
        //     30,
        //     1d,
        //     1d
        // )
        new CameraConfig( // thriftycam0
            () -> new Pose3d(
                -0.2794,
                0.2286,
                0.21113,
                new Rotation3d(
                    Degrees.of(0d),
                    Degrees.of(30d),
                    Degrees.of(180d))),
            "0x3220:0x5C8:0xA00",
            1280,
            720,
            0,
            50,
            10,
            1d,
            1d
        ),
        new CameraConfig( // thriftycam1
            () -> new Pose3d(
                -0.2794,
                0.2286,
                0.21113,
                new Rotation3d(
                    Degrees.of(0d),
                    Degrees.of(30d),
                    Degrees.of(180d))),
            "0x3220:0x5C8:0xA00",
            1280,
            720,
            0,
            50,
            10,
            1d,
            1d
        ),
        new CameraConfig( // thriftycam2
            () -> new Pose3d(
                -0.2794,
                0.2286,
                0.21113,
                new Rotation3d(
                    Degrees.of(0d),
                    Degrees.of(30d),
                    Degrees.of(180d))),
            "0x3220:0x5C8:0xA00",
            1280,
            720,
            0,
            50,
            10,
            1d,
            1d
        ),
        new CameraConfig( // thriftycam3
            () -> new Pose3d(
                -0.2794,
                0.2286,
                0.21113,
                new Rotation3d(
                    Degrees.of(0d),
                    Degrees.of(30d),
                    Degrees.of(180d))),
            "0x3220:0x5C8:0xA00",
            1280,
            720,
            0,
            50,
            10,
            1d,
            1d
        ),
    };
}
