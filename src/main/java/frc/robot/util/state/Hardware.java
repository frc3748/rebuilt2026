package frc.robot.util.state;

public interface Hardware {
    void read();

    default void write() {}
}
