package frc.robot.util;

import java.util.List;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.PathPlannerPath;

import choreo.trajectory.EventMarker;

public interface IPathCallback {
    public class PathMarkup {
        public List<ConstraintsZone> constraintsZones;
        public List<EventMarker> eventMarkers;
    }

    boolean markupUpdated();

    public PathMarkup apply(PathPlannerPath path);
}