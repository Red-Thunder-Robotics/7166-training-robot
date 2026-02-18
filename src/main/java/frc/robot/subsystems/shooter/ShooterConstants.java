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
    
    public static final int flywheelMotorIdLeft = -1;
    public static final int flywheelMotorIdMiddleUpper = -1;
    public static final int flywheelMotorIdMiddleLower = -1;
    public static final int flywheelMotorIdRight = -1;
    public static final int flywheelCurrentLimit = 40;
    public static final NeutralModeValue flywheelNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue flywheelInvertedLeft = InvertedValue.Clockwise_Positive;
    public static final InvertedValue flywheelInvertedMiddleUpper = InvertedValue.Clockwise_Positive;
    public static final InvertedValue flywheelInvertedMiddleLower = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue flywheelInvertedRight = InvertedValue.CounterClockwise_Positive;
    // public static final double flywheelOutput = 1d;
    public static final boolean flywheelFOC = true;

    public static final double flywheelPidP = 0.5d;
    public static final double flywheelFreeSpeed = flywheelFOC ? 5800d : 6000d;
    public static final double flywheelPidV = 12d / (flywheelFreeSpeed / 60d);
    public static final Distance flywheelRadius = Inches.of(2d);

    public static final int hoodMotorId = -1;
    public static final double hoodCurrentLimit = 40;
    public static final double hoodMotorReduction = (50d / 12d) * (30d / 16d) * (155d / 10d);
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
    public static record InterpolationShooterParams(double rpm, double degrees) { };

    public static enum InterpolationParamMap {
        Normal,
        Low;

        public final InterpolatingTreeMap<Double, InterpolationShooterParams> map =
            new InterpolatingTreeMap<Double, InterpolationShooterParams>(
                InverseInterpolator.forDouble(),
                (InterpolationShooterParams start, InterpolationShooterParams end, double q) ->
                    new InterpolationShooterParams(
                        MathUtil.interpolate(start.rpm, end.rpm, q),
                        MathUtil.interpolate(end.degrees, end.degrees, q)
                    )
            );

        static {
            Normal.map.put(1d, new InterpolationShooterParams(1500d, 60d));
            Normal.map.put(2d, new InterpolationShooterParams(1800d, 60d));

            // TODO: low map, if we want to use it
        }
    }
    public static final InterpolationParamMap paramMap = InterpolationParamMap.Normal;
}
