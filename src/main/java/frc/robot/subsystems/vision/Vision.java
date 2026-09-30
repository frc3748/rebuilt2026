package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.util.state.StateMachine;

public class Vision extends StateMachine<Vision.State> {
    public enum State {
        UNDETERMINED,
        APRIL_TAGS,
        OBJECTS
    }

    private final RobotState robotState;
    private final Camera[] cameras;
    private final List<DetectedObject> objects = new ArrayList<>();
    private boolean estimating = true;

    public Vision(RobotState robotState, CameraConfig... configs) {
        this(robotState, Arrays.stream(configs).map(config -> Camera.of(config, robotState)).toArray(Camera[]::new));
    }

    public Vision(RobotState robotState, Camera... cameras) {
        super("Vision", State.UNDETERMINED, State.class);
        this.robotState = robotState;
        this.cameras = cameras;
        allowAllTransitions();

        SmartDashboard.putData("Vision Disable", Commands.runOnce(() -> estimating = false)
                .ignoringDisable(true)
                .withName("Vision Disable"));
        enable();
    }

    @Override
    protected void applyState(State state) {
        boolean detecting = state == State.OBJECTS;
        double now = Timer.getFPGATimestamp();
        objects.removeIf(object -> now - object.timestamp() > VisionConstants.kObjectMemorySeconds);

        for (Camera camera : cameras) {
            camera.update(robotState, detecting);
            if (estimating) {
                camera.getMeasurements().forEach(robotState::addVisionMeasurement);
            }
            objects.addAll(camera.getObjects());
        }

        Logger.recordOutput("Vision/Estimating", estimating);
        Logger.recordOutput("Vision/Objects", objects.stream()
                .map(DetectedObject::fieldPosition)
                .toArray(Translation2d[]::new));
    }

    public List<DetectedObject> getObjects() {
        return objects;
    }

    public Optional<DetectedObject> getClosestObject() {
        Translation2d robot = robotState.getLatestFieldToRobot().getValue().getTranslation();
        return objects.stream().min(Comparator.comparingDouble(object -> object.fieldPosition().getDistance(robot)));
    }

    public Camera[] getCameras() {
        return cameras;
    }

    @Override
    protected void determineSelf() {
        setState(State.APRIL_TAGS);
    }
}
