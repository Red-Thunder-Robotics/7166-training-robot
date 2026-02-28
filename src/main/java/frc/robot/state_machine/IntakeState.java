package frc.robot.state_machine;

public enum IntakeState {
    HomeOff,
    DeployedOff,
    DeployedOn,
    DeployedReverse;

    public boolean areRollersPowered() {
        switch (this) {
            case DeployedOn:
            case DeployedReverse:
                return true;

            default:
                return false;
        }
    }
    public boolean areRollersReversed() {
        switch (this) {
            case DeployedReverse:
                return true;

            default:
                return false;
        }
    }
    public boolean isDeployed() {
        switch (this) {
            case DeployedOff:
            case DeployedOn:
            case DeployedReverse:
                return true;

            default:
                return false;
        }
    }
    public IntakeState off() {
        switch (this) {
            case DeployedOff:
            case DeployedOn:
            case DeployedReverse:
                return DeployedOff;
            default:
                return this;
        }
    }
    public IntakeState on() {
        switch (this) {
            case DeployedOff:
            case DeployedOn:
            case DeployedReverse:
                return DeployedOn;
            default:
                return this;
        }
    }
    public IntakeState reverse() {
        switch (this) {
            case DeployedOff:
            case DeployedOn:
            case DeployedReverse:
                return DeployedReverse;
            default:
                return this;
        }
    }
}
