package frc.robot.state_machine;

public enum ShooterState {
    Idle,
    Shooting;

    public boolean indexerWaitsForShooter() {
        switch (this) {
            case Idle:
                return false;
            default:
                return true;
        }
    }
}
