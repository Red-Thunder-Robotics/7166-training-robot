package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        // flywheel
        double flywheelTargetVelocityRPS;

        double flywheelMotorLeftDutyCycle;
        double flywheelMotorLeftVelocityRPS;
        double flywheelMotorLeftCurrentAmps;

        double flywheelMotorMiddleUpperDutyCycle;
        double flywheelMotorMiddleUpperVelocityRPS;
        double flywheelMotorMiddleUpperCurrentAmps;

        double flywheelMotorMiddleLowerDutyCycle;
        double flywheelMotorMiddleLowerVelocityRPS;
        double flywheelMotorMiddleLowerCurrentAmps;

        double flywheelMotorRightDutyCycle;
        double flywheelMotorRightVelocityRPS;
        double flywheelMotorRightCurrentAmps;

        // hood
        double hoodTargetPositionRotations;
        double hoodTargetPositionDegrees;

        double hoodPositionRotations;
        double hoodPositionDegrees;

        double hoodMotorDutyCycle;
        double hoodMotorVelocityRPS;
        double hoodMotorCurrentAmps;

        // kicker
        double kickerMotorDutyCycle;
        double kickerMotorVelocityRPS;
        double kickerMotorCurrentAmps;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void idle() {}

    public default void flywheelDutyCycle(double output) {}
    public default void flywheelVelocity(AngularVelocity velocity) {}
    public default void flywheelStop() {}

    public default void setHoodPosition(Angle angle) {}

    public default void kickerDutyCycle(double output) {}
    public default void kickerStop() {}
}
