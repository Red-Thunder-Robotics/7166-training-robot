package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static frc.robot.util.ConversionUtil.angleToMechanismPosition;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;

public final class ShooterConstants {
    // public static final long spinUpDurationMilliseconds = 800L;
    public static final double shouldIndexVelocityThresholdRPS = 5d;
    public static final double latencyCompensationSeconds = 0.150d;
    
    public static final int flywheelMotorId = -1;
    public static final int flywheelCurrentLimit = 40;
    public static final NeutralModeValue flywheelNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue flywheelInverted = InvertedValue.Clockwise_Positive;
    // public static final double flywheelOutput = 1d;

    public static final double flywheelPidP = 100d;
    public static final Distance flywheelRadius = Inches.of(2d);

    public static final int hoodMotorId = -1;
    public static final double hoodCurrentLimit = 40;
    public static final double hoodMotorReduction = 1d;
    public static final NeutralModeValue hoodNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue hoodInverted = InvertedValue.Clockwise_Positive;

    public static final double hoodPositionHome = angleToMechanismPosition(Degrees.of(0d));

    public static final double hoodPidP = 0d;
    public static final double hoodTargetAcceleration = 0d;
    public static final double hoodMaxVelocity = 0d;
    
    public static final int kickerMotorId = -1;
    public static final int kickerCurrentLimit = 40;
    public static final NeutralModeValue kickerNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue kickerInverted = InvertedValue.Clockwise_Positive;
    public static final double kickerOutput = 1d;

    // https://blog.eeshwark.com/robotblog/shooting-on-the-fly-pt2
    public static record ShooterParams(double rpm, double degrees) { };

    public static enum ParamMap {
        Normal,
        Low;

        public final InterpolatingTreeMap<Double, ShooterParams> map =
            new InterpolatingTreeMap<Double, ShooterParams>(
                InverseInterpolator.forDouble(),
                (ShooterParams start, ShooterParams end, double q) ->
                    new ShooterParams(
                        MathUtil.interpolate(start.rpm, end.rpm, q),
                        MathUtil.interpolate(end.degrees, end.degrees, q)
                    )
            );

        static {
            Normal.map.put(1d, new ShooterParams(1500d, 60d));
            Normal.map.put(2d, new ShooterParams(1800d, 60d));

            // TODO: low map, if we want to use it
        }
    }
    public static final ParamMap paramMap = ParamMap.Normal;
}
