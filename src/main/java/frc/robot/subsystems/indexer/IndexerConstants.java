package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class IndexerConstants {
    public static final int generalFeedMotorId = -1;
    public static final int generalFeedCurrentLimit = 40;
    public static final NeutralModeValue generalFeedNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue generalFeedInverted = InvertedValue.Clockwise_Positive;
    public static final double generalFeedOutput = 1d;
    public static final double generalFeedOutputReverse = -generalFeedOutput;

    public static final int shooterFeedMotorId = -1;
    public static final int shooterFeedCurrentLimit = 40;
    public static final NeutralModeValue shooterFeedNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue shooterFeedInverted = InvertedValue.Clockwise_Positive;
    public static final double shooterFeedOutput = 1d;
    public static final double shooterFeedOutputReverse = -shooterFeedOutput;
}
