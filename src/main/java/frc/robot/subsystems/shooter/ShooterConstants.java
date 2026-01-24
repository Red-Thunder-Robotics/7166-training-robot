package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.util.ConversionUtil.angleToMechanismPosition;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ShooterConstants {
    // public static final long spinUpDurationMilliseconds = 800L;
    public static final double shouldIndexVelocityThresholdRPS = (4500d / 60d);
    public static final double latencyCompensationSeconds = 0.150d;
    
    public static final int flywheelMotorId = -1;
    public static final int flywheelCurrentLimit = 40;
    public static final NeutralModeValue flywheelNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue flywheelInverted = InvertedValue.Clockwise_Positive;
    public static final double flywheelOutput = 1d;

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
}
