package frc.robot.subsystems.light_emitting_diodes;

public interface LightEmittingDiodesIO {
    public default void solidWhite() {}
    public default void solidRed() {}

    public default void solidGreen() {}
    public default void flashingGreen() {}
    
    public default void solidBlue() {}
    public default void flashingBlue() {}
}
