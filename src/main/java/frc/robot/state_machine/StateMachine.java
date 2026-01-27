package frc.robot.state_machine;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.ground_intake.GroundIntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class StateMachine {
    public static Alliance ALLIANCE = null;

    private static IntakeState intakeState = IntakeState.HomeOff;

    public static IntakeState getIntakeState() {
        return intakeState;
    }
    public static void setIntakeState(IntakeState newIntakeState) {
        intakeState = newIntakeState;
        GroundIntakeSubsystem.instance.stateUpdate(intakeState);
        // IndexerSubsystem.instance.intakeStateUpdate(intakeState);
    }

    private static TurretState turretState = TurretState.Idle;

    public static TurretState getTurretState() {
        return turretState;
    }
    public static void setTurretState(TurretState newTurretState) {
        turretState = newTurretState;
        TurretSubsystem.instance.stateUpdate(turretState);
    }

    private static ShooterState shooterState = ShooterState.Idle;

    public static ShooterState getShooterState() {
        return shooterState;
    }
    public static void setShooterState(ShooterState newShooterState) {
        shooterState = newShooterState;
        ShooterSubsystem.instance.stateUpdate(newShooterState);
    }

    private static ClimberState climberState = ClimberState.Idle;

    public static ClimberState getClimberState() {
        return climberState;
    }
    public static void setClimberState(ClimberState newClimberState) {
        climberState = newClimberState;
        ClimberSubsystem.instance.stateUpdate(climberState);
    }

    public static final class RobotCommands {
        // intake
        public static Command setIntakeState(IntakeState intakeState) {
            return GroundIntakeSubsystem.instance.runOnce(
                () -> StateMachine.setIntakeState(intakeState)
            );
        }

        public static Command deployIntake() {
            return setIntakeState(IntakeState.DeployedOn);
        }
        public static Command retractIntake() {
            return setIntakeState(IntakeState.HomeOff);
        }

        // turret
        public static Command setTurretState(TurretState turretState) {
            return TurretSubsystem.instance.runOnce(
                () -> StateMachine.setTurretState(turretState)
            );
        }

        public static Command setTurretIdle() {
            return setTurretState(TurretState.Idle);
        }
        public static Command setTurretHubTracking() {
            return setTurretState(TurretState.HubTracking);
        }
        public static Command setTurretAllianceFeed() {
            return setTurretState(TurretState.AllianceFeed);
        }

        // shooter
        public static Command setShooterState(ShooterState shooterState) {
            return ShooterSubsystem.instance.runOnce(
                () -> StateMachine.setShooterState(shooterState)
            );
        }

        public static Command setShooterIdle() {
            return setShooterState(ShooterState.Idle);
        }
        public static Command setShooterShooting() {
            return setShooterState(ShooterState.Shooting);
        }

        // shooter + turret
        public static Command engageShooterHub() {
            return setTurretHubTracking()
                .andThen(
                    Commands.waitUntil(() -> TurretSubsystem.instance.shouldIndex())
                        .andThen(setShooterShooting())
                );
        }
        public static Command disengageShooter() {
            return setTurretIdle()
                .alongWith(setShooterIdle());
        }

        // climber
        public static Command setClimberState(ClimberState climberState) {
            return ClimberSubsystem.instance.runOnce(
                () -> StateMachine.setClimberState(climberState)
            );
        }

        public static Command climberStateIncrease() {
            return Commands.defer(
                () -> setClimberState(getClimberState().getNext()),
                Set.of(ClimberSubsystem.instance)
            );
        }
        public static Command climberStateDecrease() {
            return Commands.defer(
                () -> setClimberState(getClimberState().getPrevious()),
                Set.of(ClimberSubsystem.instance)
            );
        }

        // FIXME: auto climb command
        public static Command autoClimb() {
            return Commands.none();
        }
    }

    private static Rotation2d swerveRotationOffset = new Rotation2d();

    public static Rotation2d getSwerveRotationOffset() {
        return swerveRotationOffset;
    }

    public static Rotation2d initialSwerveRotation = null;

    private static boolean needsToUpdateRobot = true;
    public static synchronized void updateState(Robot robot) {
        Logger.recordOutput("StateMachine/IntakeState", intakeState);
        Logger.recordOutput("StateMachine/TurretState", turretState);
        Logger.recordOutput("StateMachine/ShooterState", shooterState);
        Logger.recordOutput("StateMachine/ClimberState", climberState);

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() != ALLIANCE) {
            needsToUpdateRobot = true;

            ALLIANCE = alliance.get();
            Logger.recordOutput("StateMachine/Alliance", ALLIANCE);

            // AprilTagConstants.update(ALLIANCE);

            if (ALLIANCE == Alliance.Blue)
                initialSwerveRotation = Rotation2d.kZero;
            else
                initialSwerveRotation = Rotation2d.kPi;

            swerveRotationOffset = initialSwerveRotation.plus(Rotation2d.kPi);

            // VisionSubsystem.update();
            // for (var value : VisionSubsystem.RelativeReefLocation.values())
            //     value.update();
            // for (var value : VisionSubsystem.CoralStationID.values())
            //     value.update();
        }

        if (needsToUpdateRobot && robot != null && ALLIANCE != null) {
            needsToUpdateRobot = false;

            // if (m_visionSubsystem != null && ALLIANCE != null)
            //     VisionSubsystem.RelativeReefLocation.postUpdate(m_visionSubsystem);
        }
    }

}
