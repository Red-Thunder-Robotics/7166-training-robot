package frc.robot.subsystems.ground_intake;

import static frc.robot.subsystems.ground_intake.GroundIntakeConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateMachine.IntakeState;

public final class GroundIntakeSubsystem extends SubsystemBase {
    public static GroundIntakeSubsystem instance = null;

    private final GroundIntakeIO m_io;
    private final GroundIntakeIOInputsAutoLogged m_inputs = new GroundIntakeIOInputsAutoLogged();

    private final Runnable m_startRoller;
    private final Runnable m_reverseRoller;
    
    public GroundIntakeSubsystem(GroundIntakeIO io) {
        instance = this;

        m_io = io;

        if (rollerUseDutyCycle) {
            m_startRoller = this::startRollerDutyCycle;
            m_reverseRoller = this::reverseRollerDutyCycle;
        } else {
            m_startRoller = this::startRollerVelocity;
            m_reverseRoller = this::reverseRollerVelocity;
        }

        retract();
        stopRoller();
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        
        Logger.processInputs("GroundIntake", m_inputs);
    }

    public void stateUpdate(IntakeState intakeState) {
        switch (intakeState) {
            case HomeOff:
                stopRoller();
                retract();
                break;
            case DeployedOff:
                stopRoller();
                deploy();
                break;
            case DeployedOn:
                m_startRoller.run();
                deploy();
                break;
            case DeployedReverse:
                m_reverseRoller.run();
                deploy();
                break;
        }
    }

    private void setIdle() {
        m_io.idle();
    }

    private void startRollerDutyCycle() {
        m_io.rollerDutyCycle(rollerOutput);
    }
    private void reverseRollerDutyCycle() {
        m_io.rollerDutyCycle(rollerOutputReverse);
    }
    private void startRollerVelocity() {
        m_io.rollerRPS(rollerOutputVelocityRPS);
    }
    private void reverseRollerVelocity() {
        m_io.rollerRPS(rollerOutputVelocityReverseRPS);
    }
    public void stopRoller() {
        m_io.rollerStop();
    }

    public void deploy() {
        m_io.setActuatorPosition(actuatorPositionDeployed);
    }
    public void retract() {
        m_io.setActuatorPosition(actuatorPositionHome);
    }
}
