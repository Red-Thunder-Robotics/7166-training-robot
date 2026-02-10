package frc.robot.subsystems.climbermark2;

import static frc.robot.subsystems.climbermark2.ClimberConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state_machine.ClimberMark2State;

public final class ClimberSubsystem extends SubsystemBase {
    public static ClimberSubsystem instance = null;

    private final ClimberIO m_io;
    private final ClimberIOInputsAutoLogged m_inputs = new ClimberIOInputsAutoLogged();

    public ClimberSubsystem(ClimberIO io) {
        instance = this;

        m_io = io;
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);

        Logger.processInputs("Climber", m_inputs);
    }

    public void stateUpdate(ClimberMark2State climberState) {
        switch (climberState) {
            case Idle:
                m_io.idle();
                break;
            case DeployedGrabHome:
                m_io.setActuatorPosition(actuatorPositionDeployed);
                m_io.setGrabPosition(grabPositionHome);
                break;
            case DeployedGrabDeployed:
                m_io.setActuatorPosition(actuatorPositionDeployed);
                m_io.setGrabPosition(grabPositionDeployed);
                break;
            case Home:
                m_io.setActuatorPosition(actuatorPositionHome);
                m_io.setGrabPosition(grabPositionHome);
                break;
        }
    }
}
