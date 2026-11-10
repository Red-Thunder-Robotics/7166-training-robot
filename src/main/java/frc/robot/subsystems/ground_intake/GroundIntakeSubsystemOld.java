package frc.robot.subsystems.ground_intake;

import static frc.robot.subsystems.ground_intake.GroundIntakeConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ZeroingCommand;
import frc.robot.state_machine.IntakeState;
import frc.robot.state_machine.StateMachine;
import frc.robot.util.ConditionWaiter;
import java.util.Objects;
import org.littletonrobotics.junction.Logger;

public final class GroundIntakeSubsystemOld extends SubsystemBase {

    public enum WantedState {
        HomeOff,
        HomeReverse,
        DeployedOff,
        DeployedOn,
        DeployedReverse,
        OscillateOff,
        OscillateOn
    }

    public enum RealState {
        HomeOff,
        HomeReverse,
        DeployedOff,
        DeployedOn,
        DeployedReverse,
        OscillateOff,
        OscillateOn,
        // Extra output combinations needed while executing C3's requests.
        HomeOn,
        OscillateReverse,
        Disabled,
        Zeroing
    }

    public static GroundIntakeSubsystemOld instance = null;

    private final GroundIntakeIO m_io;
    private final GroundIntakeIOInputsAutoLogged m_inputs = new GroundIntakeIOInputsAutoLogged();

    private WantedState m_wantedState = WantedState.HomeOff;
    private RealState m_realState = RealState.Disabled;

    private ConditionWaiter m_startRollerWaiter = new ConditionWaiter(this::isAtDeployedPosition);
    private ConditionWaiter m_reverseRollerWaiter = new ConditionWaiter(this::isAtDeployedPosition);
    private ConditionWaiter m_deployWaiter = new ConditionWaiter(this::areRollersStopped);
    private ConditionWaiter m_retractWaiter = new ConditionWaiter(this::isIntakeHome);

    private boolean m_rollerForward = false;
    private boolean m_rollerHome = false;
    private boolean m_rollerReverse = false;

    // private static final double oscillateFrequencySeconds = 0.7d;
    // private static final double oscillateStartDelaySeconds = 1d; // 1
    // // private static final int oscillateCountStopThreshold = 6;
    // private static final int oscillateCountStopThreshold = 1;
    // private static final boolean oscillateStopAfterCounInTeleop = false;
    // private Timer m_oscillatorTimer = new Timer();
    // private int m_oscillateCount = 0;

    private Timer m_pushTimer = new Timer();

    private enum PushStep {
        OutOne,
        // HalfwayTwo,
        InThree
    }

    private PushStep m_pushStep = PushStep.OutOne;

    public GroundIntakeSubsystemOld(GroundIntakeIO io) {
        instance = this;

        m_io = io;
    }

    public void setWantedState(WantedState wantedState) {
        m_wantedState = Objects.requireNonNull(wantedState);
    }

    public WantedState getWantedState() {
        return m_wantedState;
    }

    public RealState getRealState() {
        return m_realState;
    }

    @SuppressWarnings("unused")
    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);

        Logger.processInputs("GroundIntake", m_inputs);

        if (DriverStation.isDisabled()) {
            // Clear requests so enabling cannot restart a previously running roller.
            StateMachine.setIntakeState(IntakeState.HomeOff);
            m_pushTimer.stop();
            m_realState = RealState.Disabled;
        } else if (ZeroingCommand.isSubsystemZeroing(this)) {
            StateMachine.setIntakeState(IntakeState.HomeOff);
            m_pushTimer.stop();
            m_realState = RealState.Zeroing;
        }

        applyOutputs();
        Logger.recordOutput("GroundIntake/WantedState", getRealState());
        Logger.recordOutput("GroundIntake/SystemState", getRealState());
        Logger.recordOutput("GroundIntake/AtHome", isIntakeHome());
        Logger.recordOutput("GroundIntake/AtDeployed", isAtDeployedPosition());

        final boolean shouldPush = StateMachine.isShooting();
        if (m_pushTimer.isRunning()) {
            if (shouldPush) {
                final boolean advance =
                        m_pushTimer.advanceIfElapsed(isTeleop() ? pushFrequencySeconds : pushFrequencySecondsAuto);
                switch (m_pushStep) {
                    case OutOne:
                        if (isAtDeployedPosition()) {
                            StateMachine.setIntakeState(IntakeState.DeployedOn);
                        }
                        if (advance) m_pushStep = PushStep.InThree; // HalfwayTwo
                        break;
                        // case HalfwayTwo:
                        //     StateMachine.setIntakeState(IntakeState.OscillateOff);
                        //     if (advance)
                        //         m_pushStep = PushStep.InThree;
                        //     break;
                    case InThree:
                        StateMachine.setIntakeState(IntakeState.HomeOff);
                        break;
                }
            } else m_pushTimer.stop();
        } else if (shouldPush) {
            m_pushTimer.reset();
            m_pushTimer.start();
            m_pushStep = PushStep.OutOne;
        }
    }

    // Dertermines what state it should be in and what it will be if you input stuff
    // Making the State machine say if you want this state then these states first or after
    private RealState dertermineRealState() {
        return switch (m_wantedState) {
            case HomeOff -> {
                if (isIntakeHome()) yield RealState.HomeOff;

                yield switch (m_realState) {
                    case DeployedOff,
                            DeployedOn,
                            DeployedReverse,
                            OscillateOff,
                            OscillateOn,
                            OscillateReverse,
                            HomeOn -> RealState.HomeOn;
                    default -> RealState.HomeOff;
                };
            }
            case HomeReverse -> isIntakeHome() ? RealState.HomeReverse : RealState.HomeOff;
            case DeployedOff -> RealState.DeployedOff;
            case DeployedOn -> RealState.DeployedOn;
            case DeployedReverse -> isAtDeployedPosition() ? RealState.DeployedReverse : RealState.DeployedOff;
            case OscillateOff -> RealState.OscillateOff;
            case OscillateOn -> {
                if (isAtDeployedPosition()) yield RealState.OscillateOn;
                yield switch (m_realState) {
                    case DeployedOn, HomeOn, OscillateOn -> RealState.OscillateOn;
                    case HomeReverse, DeployedReverse, OscillateReverse -> RealState.OscillateReverse;
                    default -> RealState.OscillateOff;
                };
            }
        };
    }

    // Acully sets what the states do
    private void applyOutputs() {
        switch (m_realState) {
            case Disabled -> {
                m_io.actuatorStop();
                stopRoller();
            }
            case Zeroing -> stopRoller();
            case HomeOff -> {
                retract();
                stopRoller();
            }
            case HomeOn -> {
                retract();
                startRoller();
            }
            case HomeReverse -> {
                retract();
                reverseRoller();
            }
            case DeployedOff -> {
                deploy();
                stopRoller();
            }
            case DeployedOn -> {
                deploy();
                startRoller();
            }
            case DeployedReverse -> {
                deploy();
                reverseRoller();
            }
            case OscillateOff -> {
                deployOscillate();
                stopRoller();
            }
            case OscillateOn -> {
                deployOscillate();
                startRoller();
            }
            case OscillateReverse -> {
                deployOscillate();
                reverseRoller();
            }
        }
    }

    public boolean isTeleop() {
        return DriverStation.isTeleop();
    }

    public Command zeroMechanisms(boolean skipDrive) {
        return skipDrive
                ? Commands.runOnce(m_io::actuatorZero, this)
                : new ZeroingCommand(
                        this,
                        m_io::actuatorZeroingDrive,
                        m_io::actuatorCanZero,
                        m_io::actuatorStop,
                        m_io::actuatorZero);
    }

    public boolean isAtDeployedPosition() {
        return m_inputs.isDeployed
                && Math.abs(m_inputs.actuatorPositionInches - m_inputs.targetActuatorPositionInches) < 0.5d;
    }

    public boolean isIntakeHome() {
        return !m_inputs.isDeployed
                && Math.abs(m_inputs.actuatorPositionInches - m_inputs.targetActuatorPositionInches) < 0.5d;
    }

    public boolean areRollersStopped() {
        return Math.abs(m_inputs.rightRollerMotorVelocityRPS) < 10d;
    }

    public void stateUpdate(IntakeState intakeState, IntakeState oldIntakeState) {
        switch (intakeState) {
            case HomeOff:
                retract();
                if (oldIntakeState.isOut()) {
                    RollerHome();
                    m_retractWaiter.activate();
                } else {
                    m_retractWaiter.activate();
                    stopRoller();
                }
                break;
            case HomeReverse:
                retract();
                if (oldIntakeState.isOut()) {
                    stopRoller();
                    m_reverseRollerWaiter.activate();
                } else reverseRoller();
                break;
            case DeployedOff:
                stopRoller();
                deploy();
                break;
            case DeployedOn:
                deploy();
                // if (oldIntakeState.isDeployed())
                //     startRoller();
                // else
                //     m_startRollerWaiter.activate();
                startRoller();
                break;
            case DeployedReverse:
                deploy();
                if (oldIntakeState.isDeployed()) reverseRoller();
                else m_reverseRollerWaiter.activate();
                break;
            case OscillateOff:
                stopRoller();
                deployOscillate();
                // stopRoller();
                break;
            case OscillateOn:
                if (isAtDeployedPosition()) {
                    startRoller();
                }
                deployOscillate();
                break;
        }
    }

    // private void setIdle() {
    //     m_io.idle();
    // }

    private void startRoller() {
        // m_io.rollerVelocity(rollerOutputVelocity);
        m_rollerForward = true;
    }

    private void reverseRoller() {
        // m_io.rollerVelocity(rollerOutputVelocityReverse);
        m_rollerReverse = true;
    }

    private void RollerHome() {
        // m_io.rollerVelocity(rollerOutputVelocityReverse);
        m_rollerHome = true;
    }

    public void stopRoller() {
        m_rollerHome = false;
        m_rollerForward = false;
        m_rollerReverse = false;
        m_io.rollerStop();
    }

    public void deploy() {
        m_io.setActuatorPosition(actuatorPositionDeployed);
    }

    public void deployOscillate() {
        m_io.setActuatorPosition(actuatorPositionOscillate);
    }

    public void retract() {
        m_io.setActuatorPosition(actuatorPositionHome);
    }
}
