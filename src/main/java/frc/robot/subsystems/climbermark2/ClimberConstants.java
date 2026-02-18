package frc.robot.subsystems.climbermark2;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeters;
import static frc.robot.util.ConversionUtil.angleToMechanismPosition;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;

public final class ClimberConstants {
    public static final int actuatorMotorId = -1;
    public static final double actuatorCurrentLimit = 40;
    public static final double actuatorMotorReduction = 1d;
    public static final NeutralModeValue actuatorNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue actuatorInverted = InvertedValue.Clockwise_Positive;

    public static final double actuatorPositionHome = angleToMechanismPosition(Degrees.of(0d));
    public static final double actuatorPositionDeployed = angleToMechanismPosition(Degrees.of(0d));

    public static final double actuatorPidP = 0d;
    public static final double actuatorTargetAcceleration = 0d;
    public static final double actuatorMaxVelocity = 0d;

    public static final int grabLeaderId = -1;
    public static final double grabLeaderCurrentLimit = 40;
    public static final double grabLeaderMotorReduction = 34.5d;
    public static final Distance grabLeaderPitchCircumference = Millimeters.of(47.75d);
    public static final NeutralModeValue grabLeaderNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue grabLeaderInverted = InvertedValue.Clockwise_Positive;

    public static final int grabFollowerId = -1;
    public static final boolean grabFollowerMatchLeaderInverted = false;

    public static final double grabPositionHome = angleToMechanismPosition(Degrees.of(0d));
    public static final double grabPositionDeployed = angleToMechanismPosition(Degrees.of(0d));

    public static final double grabPidP = 0d;
    public static final double grapTargetAcceleration = 0d;
    public static final double grapMaxVelocity = 0d;
}
