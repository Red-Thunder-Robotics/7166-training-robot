package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state_machine.RobotEvent;
import frc.robot.state_machine.ShooterState;
import frc.robot.state_machine.StateMachine;
import java.util.Objects;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public final class IndexerSubsystem extends SubsystemBase {
    public enum WantedState {
        Idle,
        Shooting,
        Reverse
    }

    public enum RealState {
        Idle,
        Shooting,
        Reverse,
        PreShooting,
        Disabled
    }

    public static IndexerSubsystem instance = null;

    private final IndexerIO m_io;
    private final IndexerIOInputsAutoLogged m_inputs = new IndexerIOInputsAutoLogged();
    private static WantedState m_wantedState = WantedState.Idle;
    private static RealState m_RealState = RealState.Idle;

    private boolean m_isFeeding;

    private Timer m_reverseTimer = new Timer();

    public IndexerSubsystem(IndexerIO io) {
        instance = this;

        m_io = io;

        RobotEvent.OnShootingStart.addListener(() -> {
            m_io.topRollerVelocity(topRollerVelocityReverse);
            m_reverseTimer.restart();
        });
        RobotEvent.OnShootingEnd.addListener(() -> {
            m_reverseTimer.stop();
        });
    }

    public static void setWantedState(WantedState wantedState) {
        m_wantedState = Objects.requireNonNull(wantedState);
    }

    public static WantedState getWantedState() {
        return m_wantedState;
    }

    public static RealState getRealState() {
        return m_RealState;
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);

        Logger.processInputs("Indexer", m_inputs);

        // if (StateMachine.getWantsReverseIndexerBeforeShoot()) {
        //     m_io.indexerVelocity(indexerOutputVelocityReverse);
        //     m_io.topRollerVelocity(topRollerVelocityReverse);
        // } else
        // if (StateMachine.getIndexerShooterStuff()) {
        //     m_io.indexerVelocity(indexerOutputVelocity);
        //     m_io.topRollerVelocity(topRollerVelocity);
        // }

        if (DriverStation.isDisabled()) {
            // Clear requests so enabling cannot restart a previously running roller.
            StateMachine.setShooterState(ShooterState.Idle);
            m_RealState = RealState.Disabled;
        } else {
            m_RealState = determineRealState();
        }

        applyOutputs();
        Logger.recordOutput("Indexer/WantedState", m_wantedState);
        Logger.recordOutput("Indexer/SystemState", m_RealState);
    }

    private RealState determineRealState() {
        return switch (m_wantedState) {
            case Idle -> RealState.Idle;
            case Reverse -> RealState.Reverse;
            case Shooting -> switch (m_RealState) {
                case PreShooting -> m_reverseTimer.hasElapsed(reverseTime) ? RealState.Shooting : RealState.PreShooting;
                case Shooting -> RealState.Shooting;
                default -> RealState.PreShooting; // from Idle, Reverse or Disabled
            };
        };
    }

    public void applyOutputs() {
        boolean isFeeding = false;
        switch (m_RealState) {
            case Disabled -> {
                m_io.topRollerStop();
                m_io.indexerStop();
                m_io.lowerKickerStop();
            }
            case Idle -> setIdle();
            case Shooting -> {
                if (StateMachine.shouldIndex()) {
                    m_isFeeding = true;
                    m_io.indexerVelocity(indexerOutputVelocity);
                    m_io.topRollerVelocity(topRollerVelocity);
                } else {
                    m_io.indexerStop();
                    m_io.topRollerStop();
                }
                m_io.lowerKickerVelocity(lowerKickerVelocity); // runs either way
            }

            case Reverse -> {
                m_io.indexerVelocity(indexerOutputVelocityReverse);
                m_io.topRollerVelocity(topRollerVelocityReverse);
                m_io.lowerKickerVelocity(lowerKickerVelocityReverse);
                break;
            }
            case PreShooting -> {
                m_io.indexerStop();
                m_io.lowerKickerStop();
                m_io.topRollerVelocity(topRollerVelocityReverse);
            }
        }
        m_isFeeding = isFeeding;
    }

    @AutoLogOutput(key = "IndexerIsFeeding")
    public boolean getIsFeeding() {
        return m_isFeeding;
    }

    @AutoLogOutput(key = "IndexerPastThreshold")
    public boolean getIsPastThreshold() {
        final double lowerKickerTarget = m_inputs.lowerKickerVelocityRPS;
        if (lowerKickerTarget == 0d) return false;

        final boolean lowerKicker =
                Math.abs(lowerKickerTarget - m_inputs.lowerKickerTargetVelocityRPS) <= lowerKickerVelocityThresholdRPS;

        return lowerKicker;
    }

    @AutoLogOutput(key = "ReverseDone")
    public boolean getReverseIsDone() {
        return m_reverseTimer.hasElapsed(reverseTime);
    }

    public void setIdle() {
        m_io.indexerStop();
        m_io.topRollerStop();
        m_io.lowerKickerStop();
    }
}
