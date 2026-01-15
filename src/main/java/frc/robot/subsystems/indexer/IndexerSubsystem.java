package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateMachine;
import frc.robot.StateMachine.IntakeState;

public final class IndexerSubsystem extends SubsystemBase {
    public static IndexerSubsystem instance = null;

    private final IndexerIO m_io;
    private final IndexerIOInputsAutoLogged m_inputs = new IndexerIOInputsAutoLogged();

    public IndexerSubsystem(IndexerIO io) {
        instance = this;

        m_io = io;
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);

        Logger.processInputs("Indexer", m_inputs);
    }

    public void stateUpdate(boolean indexerEnabled) {
        if (indexerEnabled) {
            m_io.generalFeedDutyCycle(generalFeedOutput);
            m_io.shooterFeedDutyCycle(shooterFeedOutput);
        } else {
            m_io.generalFeedStop();
            m_io.shooterFeedStop();
        }
    }

    public void intakeStateUpdate(IntakeState intakeState) {
        // FIXME: do we want to automatically reverse indexer on intake reverse? if so comment back in
        // if (intakeState.areRollersReversed()) {
        //     StateMachine.setIndexerEnabled(false);
        //     m_io.generalFeedDutyCycle(generalFeedOutputReverse);
        //     m_io.shooterFeedDutyCycle(shooterFeedOutputReverse);
        // } else
            StateMachine.setIndexerEnabled(!intakeState.areRollersPowered());
    }
}
