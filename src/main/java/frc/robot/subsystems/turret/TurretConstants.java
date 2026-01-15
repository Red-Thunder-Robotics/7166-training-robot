package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.util.ConversionUtil.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class TurretConstants {
    public static final int motorId = -1;
    public static final double currentLimit = 40;
    public static final double motorReduction = 1d;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final InvertedValue inverted = InvertedValue.Clockwise_Positive;

    public static final double positionMin = angleToMechanismPosition(Degrees.of(-200d));
    public static final double positionMax = angleToMechanismPosition(Degrees.of(200d));
    public static final double positionHome = angleToMechanismPosition(Degrees.of(0d));

    public static final double pidP = 0d;
    public static final double targetAcceleration = 0d;
    public static final double maxVelocity = 0d;
}
