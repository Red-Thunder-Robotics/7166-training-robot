package frc.robot.state_machine;

public enum ShooterState {
    Idle,
    Shooting,
    Reversing;

    public boolean indexerWaitsForShooter() {
        switch (this) {
            case Idle:
                return false;
            default:
                return true;
        }
    }
}
