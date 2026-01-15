package frc.robot.subsystems.ground_intake;

import static frc.robot.subsystems.ground_intake.GroundIntakeConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateMachine.IntakeState;

public final class GroundIntakeSubsystem extends SubsystemBase {
    public static GroundIntakeSubsystem instance = null;

    private final GroundIntakeIO m_io;
    private final GroundIntakeIOInputsAutoLogged m_inputs = new GroundIntakeIOInputsAutoLogged();
    
    public GroundIntakeSubsystem(GroundIntakeIO io) {
        instance = this;

        m_io = io;

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
                startRoller();
                deploy();
                break;
            case DeployedReverse:
                reverseRoller();
                deploy();
                break;
        }
    }

    private void setIdle() {
        m_io.idle();
    }

    private void startRoller() {
        m_io.rollerDutyCycle(rollerOutput);
    }
    private void reverseRoller() {
        m_io.rollerDutyCycle(rollerOutputReverse);
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
