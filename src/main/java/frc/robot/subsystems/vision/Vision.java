package frc.robot.subsystems.vision;

import java.util.Arrays;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.util.state.StateMachine;

public class Vision extends StateMachine<Vision.State> {
    private final RobotState state;
    private final Camera[] cameras;

    public Vision(RobotState state, CameraConfig... configs) {
        this(state, Arrays.stream(configs).map(config -> Camera.of(config, state)).toArray(Camera[]::new));
    }

    public Vision(RobotState state, Camera... cameras) {
        super("Vision", State.UNDETERMINED, State.class);
        this.state = state;
        this.cameras = cameras;

        SmartDashboard.putData("Vision Disable", Commands.runOnce(state::disableVision)
                .ignoringDisable(true)
                .withName("Vision Disable"));
        enable();
    }

    @Override
    protected void update() {
        for (Camera camera : cameras) {
            camera.update(state).ifPresent(state::addVisionEstimate);
        }
    }

    public Camera[] getCameras() {
        return cameras;
    }

    @Override
    protected void determineSelf() {
        setState(State.SCANNING);
    }

    public enum State {
        UNDETERMINED,
        SCANNING
    }
}
