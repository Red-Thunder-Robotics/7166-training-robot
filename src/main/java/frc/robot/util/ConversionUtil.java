package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public final class ConversionUtil {
    public static double angleToMechanismPosition(Angle angle) {
        return angle.in(Degrees) / 360d;
    }
    public static Angle mechanismPositionToAngle(double position) {
        return Degrees.of(position * 360d);
    }
}
