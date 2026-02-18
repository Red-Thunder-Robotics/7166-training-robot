package frc.robot.subsystems.climbermark2;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.climbermark2.ClimberConstants.actuatorPositionHome;
import static frc.robot.subsystems.climbermark2.ClimberConstants.grabPositionHome;
import static frc.robot.util.ConversionUtil.mechanismPositionToAngle;

import edu.wpi.first.math.controller.PIDController;

public final class ClimberIOSim implements ClimberIO {
    private double m_actuatorTargetPosition = actuatorPositionHome;
    private double m_actuatorPosition = m_actuatorTargetPosition;

    private double m_grabTargetPosition = grabPositionHome;
    private double m_grabPosition = m_grabTargetPosition;

    private final PIDController m_actuatorPID = new PIDController(0.5d, 0d, 0d);
    private final PIDController m_grabPID = new PIDController(0.5d, 0d, 0d);

    public ClimberIOSim() {

    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        // actuator
        final double actuatorOutput = m_actuatorPID.calculate(m_actuatorPosition);
        m_actuatorPosition += actuatorOutput;

        final double actuatorTargetPosition = m_actuatorTargetPosition;
        inputs.actuatorTargetPositionRotations = actuatorTargetPosition;
        inputs.actuatorTargetPositionInches = mechanismPositionToAngle(actuatorTargetPosition).in(Degrees);

        final double actuatorPosition = m_actuatorPosition;
        inputs.actuatorPositionRotations = actuatorPosition;
        inputs.actuatorPositionInches = mechanismPositionToAngle(actuatorPosition).in(Degrees);

        inputs.actuatorDutyCycle = actuatorOutput;

        // grab
        final double grabTargetPosition = m_grabTargetPosition;
        inputs.grabTargetPositionRotations = grabTargetPosition;
        inputs.grabTargetPositionDegrees = mechanismPositionToAngle(grabTargetPosition).in(Degrees);

        // grab leader
        final double grabOutput = m_grabPID.calculate(m_grabPosition);
        m_grabPosition += grabOutput;

        final double grabLeaderPosition = m_grabPosition;
        inputs.grabLeaderPositionRotations = grabLeaderPosition;
        inputs.grabLeaderPositionInches = mechanismPositionToAngle(grabLeaderPosition).in(Degrees);

        inputs.grabLeaderDutyCycle = grabOutput;
    }

    @Override
    public void setActuatorPosition(double position) {
        m_actuatorTargetPosition = position;
        m_actuatorPID.setSetpoint(position);
    }
    @Override
    public void setGrabPosition(double position) {
        m_grabTargetPosition = position;
        m_grabPID.setSetpoint(position);
    }
}
