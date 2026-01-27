package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;

public final class ConversionUtil {
    public static double angleToMechanismPosition(Angle angle) {
        return angle.in(Degrees) / 360d;
    }
    public static Angle mechanismPositionToAngle(double position) {
        return Degrees.of(position * 360d);
    }

    public static Translation2d chassisSpeedsToTranslation2d(ChassisSpeeds speeds) {
        return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }
    public static Translation3d chassisSpeedsToTranslation3d(ChassisSpeeds speeds) {
        return new Translation3d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0d);
    }
}
