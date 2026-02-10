package frc.robot.state_machine;

public enum ClimberMark2State {
    Idle,
    DeployedGrabHome,
    DeployedGrabDeployed,
    Home;

    public ClimberMark2State getNext() {
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
    public ClimberMark2State getPrevious() {
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
