package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;

public final class IndexerConstants {
    public static final int generalFeedMotorId = 56;
    // public static final int generalFeedCurrentLimit = 40;
    public static final NeutralModeValue generalFeedNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue generalFeedInverted = InvertedValue.CounterClockwise_Positive;
    // public static final double generalFeedOutput = 1d;
    // public static final double generalFeedOutputReverse = -generalFeedOutput;
    public static final AngularVelocity generalFeedOutputVelocity = RPM.of(3000d);
    public static final AngularVelocity generalFeedOutputVelocityReverse = generalFeedOutputVelocity.unaryMinus();

    public static final double generalFeedPidP = 1.38d;
    public static final double generalFeedPidV = 12d / (5800d / 60d);
    public static final double generalFeedTargetAcceleration = 266d;

    // public static final int shooterFeedMotorId = -1;
    // public static final int shooterFeedCurrentLimit = 40;
    // public static final NeutralModeValue shooterFeedNeutralMode = NeutralModeValue.Coast;
    // public static final InvertedValue shooterFeedInverted = InvertedValue.Clockwise_Positive;
    // public static final double shooterFeedOutput = 1d;
    // public static final double shooterFeedOutputReverse = -shooterFeedOutput;
}
