package frc.robot.subsystems.ground_intake;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.util.ConversionUtil.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class GroundIntakeConstants {
    public static final int rollerMotorId = -1;
    public static final int rollerCurrentLimit = 40;
    public static final NeutralModeValue rollerNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue rollerInverted = InvertedValue.Clockwise_Positive;
    public static final double rollerOutput = 0.85d;
    public static final double rollerOutputReverse = -rollerOutput;

    public static final int actuatorMotorId = -1;
    public static final int actuatorCurrentLimit = 40;
    public static final double actuatorMotorReduction = 1d;
    public static final NeutralModeValue actuatorNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue actuatorInverted = InvertedValue.Clockwise_Positive;
    public static final double actuatorPositionHome = angleToMechanismPosition(Degrees.of(0d));
    public static final double actuatorPositionDeployed = angleToMechanismPosition(Degrees.of(0d));

    public static final double actuatorPidP = 0d;
    public static final double actuatorTargetAcceleration = 0d;
    public static final double actuatorMaxVelocity = 0d;
}
