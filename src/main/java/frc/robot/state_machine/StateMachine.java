package frc.robot.state_machine;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Robot;
import frc.robot.subsystems.climbermark1.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.ground_intake.GroundIntakeSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.HubActiveState;

public class StateMachine {
    public static Alliance ALLIANCE = null;

    public static Distance allianceFlipX(Distance x) {
        return ALLIANCE == Alliance.Red ? FieldConstants.FIELD_LENGTH.minus(x) : x;
    }

    public static Distance allianceFlipY(Distance y) {
        return ALLIANCE == Alliance.Red ? FieldConstants.FIELD_WIDTH.minus(y) : y;
    }

    public static Translation2d allianceFlip(Translation2d translation) {
        return new Translation2d(allianceFlipX(translation.getMeasureX()), allianceFlipY(translation.getMeasureY()));
    }
    
    public static Rotation2d allianceFlip(Rotation2d rotation) {
        return ALLIANCE == Alliance.Red ? rotation.rotateBy(Rotation2d.kPi) : rotation;
    }

    public static Pose2d allianceFlip(Pose2d pose) {
        return ALLIANCE == Alliance.Red
            ? new Pose2d(allianceFlip(pose.getTranslation()), allianceFlip(pose.getRotation()))
            : pose;
    }

    public static Translation3d allianceFlip(Translation3d translation) {
        return new Translation3d(
            allianceFlipX(translation.getMeasureX()), allianceFlipY(translation.getMeasureY()), translation.getMeasureZ());
    }

    public static Rotation3d allianceFlip(Rotation3d rotation) {
        return ALLIANCE == Alliance.Red ? rotation.rotateBy(new Rotation3d(0d, 0d, Math.PI)) : rotation;
    }

    public static Pose3d allianceFlip(Pose3d pose) {
        return new Pose3d(allianceFlip(pose.getTranslation()), allianceFlip(pose.getRotation()));
    }

    private static GeneralRobotState generalRobotState = GeneralRobotState.Idle;
    
    public static GeneralRobotState getGeneralRobotState() {
        return generalRobotState;
    }

    private static Optional<Translation3d> shooterTargetPose = Optional.empty();
    public static Optional<Translation3d> getShooterTargetPose() {
        return shooterTargetPose;
    }

    private static IntakeState intakeState = IntakeState.HomeOff;

    public static IntakeState getIntakeState() {
        return intakeState;
    }
    public static void setIntakeState(IntakeState newIntakeState) {
        intakeState = newIntakeState;
        GroundIntakeSubsystem.instance.stateUpdate(intakeState);
        // IndexerSubsystem.instance.intakeStateUpdate(intakeState);
    }

    private static ShooterTargetState shooterTargetState = ShooterTargetState.Idle;

    public static ShooterTargetState getShooterTargetState() {
        return shooterTargetState;
    }
    public static void setShooterTargetState(ShooterTargetState newTurretState) {
        shooterTargetState = newTurretState;
        if (Constants.USE_TURRET)
            TurretSubsystem.instance.stateUpdate(shooterTargetState);
    }

    private static boolean withinJoystickRotationErrorThreshold = false;
    public static boolean getWithinJoystickRotationErrorThreshold() {
        return withinJoystickRotationErrorThreshold;
    }
    public static void setWithinJoystickRotationErrorThreshold(boolean within) {
        withinJoystickRotationErrorThreshold = within;
    }

    public static boolean turretShouldIndex() {
        if (LiveConfig.getIsPit())
            return true;
        return Constants.USE_TURRET ? TurretSubsystem.instance.shouldIndex() : withinJoystickRotationErrorThreshold;
    }
    public static boolean shouldIndex() {
        return turretShouldIndex() && ShooterSubsystem.instance.shouldIndex();
    }

    private static ShooterState shooterState = ShooterState.Idle;

    public static ShooterState getShooterState() {
        return shooterState;
    }
    public static void setShooterState(ShooterState newShooterState) {
        shooterState = newShooterState;
        ShooterSubsystem.instance.stateUpdate(newShooterState);
    }

    private static ClimberMark1State climberLeftState = ClimberMark1State.Idle;

    public static ClimberMark1State getClimberLeftState() {
        return climberLeftState;
    }
    public static void setClimberLeftState(ClimberMark1State newClimberState) {
        climberLeftState = newClimberState;
        ClimberSubsystem.instance.stateUpdateLeft(climberLeftState);
    }

    private static ClimberMark1State climberRightState = ClimberMark1State.Idle;

    public static ClimberMark1State getClimberRightState() {
        return climberRightState;
    }
    public static void setClimberRightState(ClimberMark1State newClimberState) {
        climberRightState = newClimberState;
        ClimberSubsystem.instance.stateUpdateRight(climberRightState);
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
        public static Command setShooterTargetState(ShooterTargetState turretState) {
            Command command = Commands.runOnce(() -> StateMachine.setShooterTargetState(turretState));
            if (Constants.USE_TURRET)
                command.addRequirements(TurretSubsystem.instance);
            return command;
        }

        public static Command setShooterTargetIdle() {
            return setShooterTargetState(ShooterTargetState.Idle);
        }
        public static Command setShooterTargetHubTracking() {
            return setShooterTargetState(ShooterTargetState.HubTracking);
        }
        public static Command setShooterTargetAllianceFeed() {
            return setShooterTargetState(ShooterTargetState.AllianceFeed);
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
        public static Command setShooterReversing() {
            return setShooterState(ShooterState.Reversing);
        }

        // shooter + turret
        public static Command engageShooterHub() {
            return setShooterTargetHubTracking()
                .andThen(Commands.waitUntil(StateMachine::turretShouldIndex))
                .andThen(setShooterShooting());
        }
        public static Command engageShooterAllianceFeed() {
            return setShooterTargetAllianceFeed()
                .andThen(Commands.waitUntil(StateMachine::turretShouldIndex))
                .andThen(setShooterShooting());
        }
        public static Command disengageShooter() {
            return setShooterTargetIdle()
                .alongWith(setShooterIdle());
        }

        // climber mark1
        public static Command setClimberBothState(ClimberMark1State climberState) {
            return ClimberSubsystem.instance.runOnce(
                () -> {
                    StateMachine.setClimberLeftState(climberState);
                    StateMachine.setClimberRightState(climberState);
                }
            );
        }
        public static Command setClimberLeftState(ClimberMark1State climberState) {
            return ClimberSubsystem.instance.runOnce(
                () -> StateMachine.setClimberLeftState(climberState)
            );
        }
        public static Command setClimberRightState(ClimberMark1State climberState) {
            return ClimberSubsystem.instance.runOnce(
                () -> StateMachine.setClimberRightState(climberState)
            );
        }

        public static Command climberLeftHome() {
            return setClimberLeftState(ClimberMark1State.Home);
        }
        public static Command climberLeftDeployed() {
            return setClimberLeftState(ClimberMark1State.Deployed);
        }

        public static Command climberRightHome() {
            return setClimberRightState(ClimberMark1State.Home);
        }
        public static Command climberRightDeployed() {
            return setClimberRightState(ClimberMark1State.Deployed);
        }

        // climber mark2
        // public static Command setClimberState(ClimberMark2State climberState) {
        //     return ClimberSubsystem.instance.runOnce(
        //         () -> StateMachine.setClimberState(climberState)
        //     );
        // }

        // public static Command climberStateIncrease() {
        //     return Commands.defer(
        //         () -> setClimberState(getClimberState().getNext()),
        //         Set.of(ClimberSubsystem.instance)
        //     );
        // }
        // public static Command climberStateDecrease() {
        //     return Commands.defer(
        //         () -> setClimberState(getClimberState().getPrevious()),
        //         Set.of(ClimberSubsystem.instance)
        //     );
        // }

        private static boolean hasAutoClimbRan = false;
        public static boolean getHasAutoClimbRan() {
            return hasAutoClimbRan;
        }

        // FIXME: auto climb behavior
        public static Command autoClimb() {
            return Commands.runOnce(() -> hasAutoClimbRan = true);
        }
    }

    private static Rotation2d swerveRotationOffset = new Rotation2d();

    public static Rotation2d getSwerveRotationOffset() {
        return swerveRotationOffset;
    }

    public static Rotation2d initialSwerveRotation = null;

    public static Angle getRobotRotationalGoal(Translation3d targetPose) {
        Pose2d robotPose = Drive.instance.getPose();
        final ChassisSpeeds robotSpeeds = Drive.instance.getChassisSpeeds();

        // t1
        // robotPose = robotPose.exp(new Twist2d(
        //     robotSpeeds.vxMetersPerSecond * lookAheadTime,
        //     robotSpeeds.vyMetersPerSecond * lookAheadTime,
        //     robotSpeeds.omegaRadiansPerSecond * lookAheadTime
        // ));

        // t0
        // Translation2d robotTranslation = robotPose.getTranslation();
        // Translation2d targetTranslation = targetPose.getTranslation().toTranslation2d();

        // Rotation2d targetRotation = targetTranslation.minus(robotTranslation).getAngle();
        // Rotation2d currentRotation = robotPose.getRotation();

        // Rotation2d rotationError = targetRotation.minus(currentRotation);

        // final double targetangle = rotationError.getRadians();

        // return Radians.of(targetangle);

        final Translation2d robotTranslation = robotPose.getTranslation();
        final Translation2d targetTranslation = targetPose.toTranslation2d();

        Translation2d yawTranslation = targetTranslation.minus(robotTranslation);
        // Translation2d yawTranslation = robotTranslation.minus(targetTranslation);
        yawTranslation = yawTranslation.minus(new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond));

        final Rotation2d currentRotation = robotPose.getRotation();
        final Rotation2d rotationError = yawTranslation.getAngle().minus(currentRotation);

        final double targetangle = rotationError.getRadians();

        return Radians.of(targetangle);
    }

    private static final Translation3d hubPoseBlue = new Translation3d(
        Inches.of(182.11d),
        Inches.of(158.84d),
        Feet.of(6d));
    private static Translation3d hubPose = new Translation3d();
    public static Translation3d getHubPose() {
        return hubPose;
    }

    // https://github.com/hammerheads5000/2026Rebuilt/blob/b32c384e337b1a89d3d651c05696c9366a105504/src/main/java/frc/robot/Constants.java#L398
    // see 5000-License.md
    public static final Translation3d allianceFeedLeft = new Translation3d(
        Inches.of(90), FieldConstants.FIELD_WIDTH.div(2).plus(Inches.of(85)), Inches.zero());
    public static final Translation3d allianceFeedCenter =
        new Translation3d(Inches.of(90), FieldConstants.FIELD_WIDTH.div(2), Inches.zero());
    public static final Translation3d allianceFeedRight = new Translation3d(
        Inches.of(90), FieldConstants.FIELD_WIDTH.div(2).minus(Inches.of(85)), Inches.zero());

    public static final OdometryAndVision odometryAndVision = new OdometryAndVision();

    private static boolean isRobotOnAllianceLeft() {
        final var y = Drive.instance.getPose()
                .getMeasureY();
        final var threshold = FieldConstants.FIELD_WIDTH.div(2d);

        if (ALLIANCE == Alliance.Red)
            return y.lte(threshold);
        else
            return y.gt(threshold);
    }

    private static boolean needsToUpdateRobot = true;
    @SuppressWarnings("unused") // for useTurret
    public static synchronized void periodic(Robot robot) {
        Logger.recordOutput("StateMachine/IntakeState", intakeState);
        Logger.recordOutput("StateMachine/ShooterTargetState", shooterTargetState);
        Logger.recordOutput("StateMachine/ShooterState", shooterState);
        // Logger.recordOutput("StateMachine/ClimberState", climberState);
        Logger.recordOutput("StateMachine/ClimberLeftState", climberLeftState);
        Logger.recordOutput("StateMachine/ClimberRightState", climberRightState);

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

            hubPose = allianceFlip(hubPoseBlue);
        }

        if (needsToUpdateRobot && robot != null && ALLIANCE != null) {
            needsToUpdateRobot = false;

            // if (m_visionSubsystem != null && ALLIANCE != null)
            //     VisionSubsystem.RelativeReefLocation.postUpdate(m_visionSubsystem);
        }

        if (Constants.USE_TURRET && TurretSubsystem.instance.getManualEnabled())
            shooterTargetPose = Optional.empty();
        else {
            switch (StateMachine.getShooterTargetState()) {
                case HubTracking:
                    shooterTargetPose = Optional.of(hubPose);
                    break;
                case AllianceFeed:
                    shooterTargetPose = Optional.of(isRobotOnAllianceLeft() ? allianceFeedLeft : allianceFeedRight);
                    break;
                default:
                    shooterTargetPose = Optional.empty();
                    break;
            }
        }

        {
            GeneralRobotState newGeneralRobotState = GeneralRobotState.Idle;
            final boolean isFiring = IndexerSubsystem.instance.getIsFeeding();

            switch (shooterTargetState) {
                case HubTracking:
                    newGeneralRobotState = isFiring ? GeneralRobotState.HubTrackingFiring : GeneralRobotState.HubTracking;
                    break;
                case AllianceFeed:
                    newGeneralRobotState = isFiring ? GeneralRobotState.AllianceFeedFiring : GeneralRobotState.AllianceFeed;
                    break;
                default:
                    break;
            }

            generalRobotState = newGeneralRobotState;

            Logger.recordOutput("StateMachine/GeneralRobotState", generalRobotState);
        }

        Logger.recordOutput("StateMachine/ShooterTargetPose", new Pose3d(shooterTargetPose.orElse(new Translation3d()), new Rotation3d()));

        Logger.recordOutput("StateMachine/ShouldIndex", shouldIndex());
        Logger.recordOutput("StateMachine/TurretShouldIndex", turretShouldIndex());

        HubActiveState.periodic();

        Logger.recordOutput("StateMachine/HubActive", HubActiveState.isOurHubActive());
        Logger.recordOutput("StateMachine/TimeUntilSwap", HubActiveState.timeUntilSwap());
    }

}
