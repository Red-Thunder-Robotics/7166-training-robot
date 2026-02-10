package frc.robot.subsystems.climbermark1;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import static frc.robot.util.ConversionUtil.angleToMechanismPosition;
import static frc.robot.util.ConversionUtil.distanceToMechanismPosition;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;

public final class ClimberConstants {
    public static final int leftMotorId = -1;
    public static final double leftCurrentLimit = 40;
    public static final InvertedValue leftInverted = InvertedValue.Clockwise_Positive;
    
    public static final int rightMotorId = -1;
    public static final double rightCurrentLimit = 40;
    public static final InvertedValue rightInverted = InvertedValue.Clockwise_Positive;
    
    public static final double motorReduction = 1d;
    public static final Distance pitchCircumference = Millimeters.of(0d);
    public static final NeutralModeValue neutralMode = NeutralModeValue.Coast;

    public static final double positionHome = distanceToMechanismPosition(Inches.of(0d), pitchCircumference);
    public static final double positionDeployed = distanceToMechanismPosition(Inches.of(0d), pitchCircumference);

    public static final double pidP = 0d;
    public static final double targetAcceleration = 0d;
    public static final double maxVelocity = 0d;
}
