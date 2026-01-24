package frc.robot.state_machine;

public enum ClimberState {
    Idle,
    DeployedGrabHome,
    DeployedGrabDeployed,
    Home;

    public ClimberState getNext() {
        switch (this) {
            case Idle:
            case Home:
                return DeployedGrabHome;
            case DeployedGrabHome:
                return DeployedGrabDeployed;
            default:
                return this;
        }
    }
    public ClimberState getPrevious() {
        switch (this) {
            case DeployedGrabDeployed:
                return DeployedGrabHome;
            case DeployedGrabHome:
                return Home;
            default:
                return this;
        }
    }
}
