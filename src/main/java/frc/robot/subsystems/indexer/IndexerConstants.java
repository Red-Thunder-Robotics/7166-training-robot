package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;

public final class IndexerConstants {
    public static final int motorId = 56;
    // public static final int currentLimit = 40;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Coast;
    public static final InvertedValue inverted = InvertedValue.CounterClockwise_Positive;
    public static final AngularVelocity outputVelocity = RPM.of(3000d);
    public static final AngularVelocity outputVelocityReverse = outputVelocity.unaryMinus();

    public static final double pidP = 1.38d;
    public static final double pidV = 12d / (5800d / 60d);
    public static final double targetAcceleration = 266d;
}
