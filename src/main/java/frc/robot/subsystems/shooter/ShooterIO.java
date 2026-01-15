package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        // flywheel
        double flywheelMotorDutyCycle;
        double flywheelMotorVelocityRPS;
        double flywheelMotorCurrentAmps;

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
    public default void flywheelStop() {}

    public default void setHoodPosition(double position) {}

    public default void kickerDutyCycle(double output) {}
    public default void kickerStop() {}
}
