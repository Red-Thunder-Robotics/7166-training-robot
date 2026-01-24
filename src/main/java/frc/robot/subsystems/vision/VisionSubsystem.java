package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class VisionSubsystem extends SubsystemBase {
    public static VisionSubsystem instance = null;

    private final VisionIO m_io;
    private final VisionIOInputsAutoLogged m_inputs = new VisionIOInputsAutoLogged();

    public VisionSubsystem(VisionIO io) {
        instance = this;

        m_io = io;
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);

        Logger.processInputs("Vision", m_inputs);
    }
}
