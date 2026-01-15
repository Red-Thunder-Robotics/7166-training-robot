package frc.robot.subsystems.indexer;

public final class IndexerIOSim implements IndexerIO {
    private double m_generalFeedOutput = 0d;
    private double m_shooterFeedOutput = 0d;

    public IndexerIOSim() {

    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.generalFeedMotorDutyCycle = m_generalFeedOutput;
        inputs.shooterFeedMotorDutyCycle = m_shooterFeedOutput;
    }

    @Override
    public void idle() {
        m_generalFeedOutput = 0d;
        m_shooterFeedOutput = 0d;
    }

    @Override
    public void generalFeedDutyCycle(double output) {
        m_generalFeedOutput = output;
    }
    @Override
    public void generalFeedStop() {
        m_generalFeedOutput = 0d;
    }
    @Override
    public void shooterFeedDutyCycle(double output) {
        m_shooterFeedOutput = output;
    }
    @Override
    public void shooterFeedStop() {
        m_shooterFeedOutput = 0d;
    }
}
