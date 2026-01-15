package frc.robot.subsystems.ground_intake;

import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {
    @AutoLog
    public static class GroundIntakeIOInputs {
        boolean isDeployed;

        double targetActuatorPositionRotations;
        double targetActuatorPositionDegrees;

        double actuatorPositionRotations;
        double actuatorPositionDegrees;

        double rollerMotorDutyCycle;
        double rollerMotorVelocityRPS;
        double rollerMotorCurrentAmps;
    }

    public default void updateInputs(GroundIntakeIOInputs inputs) {}

    public default void idle() {}
    
    public default void setActuatorPosition(double position) {}
    public default void rollerDutyCycle(double output) {}
    public default void rollerRPM(double rpm) {} // FIXME: code this
    public default void rollerStop() {}
}
