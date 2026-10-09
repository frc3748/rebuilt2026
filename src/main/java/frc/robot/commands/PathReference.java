package frc.robot.commands;

import java.util.Arrays;
import java.util.List;
import java.util.stream.IntStream;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class PathReference {
    private static final double kSearchBackMeters = 0.5;
    private static final double kSearchAheadMeters = 2.0;
    private static final double kMinSegmentMeters = 1e-6;

    public record Sample(Pose2d pose, double distance, double speed, double omega) {}

    public record Projection(int index, double distance, double crossTrack, Translation2d tangent) {}

    private final double[] time;
    private final double[] x;
    private final double[] y;
    private final double[] distance;
    private final double[] heading;
    private final double[] speed;

    private PathReference(double[] time, double[] x, double[] y, double[] heading, double[] speed) {
        this.time = time;
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.speed = speed;
        distance = new double[time.length];
        for (int i = 1; i < time.length; i++) {
            distance[i] = distance[i - 1] + Math.hypot(x[i] - x[i - 1], y[i] - y[i - 1]);
        }
    }

    public static PathReference build(PathPlannerTrajectory trajectory, double maxTurnRate) {
        List<PathPlannerTrajectoryState> states = trajectory.getStates();
        int n = states.size();
        double[] time = new double[n];
        double[] speed = new double[n];
        double[] x = new double[n];
        double[] y = new double[n];
        Rotation2d[] headings = new Rotation2d[n];
        for (int i = 0; i < n; i++) {
            PathPlannerTrajectoryState state = states.get(i);
            time[i] = state.timeSeconds;
            speed[i] = state.linearVelocity;
            x[i] = state.pose.getX();
            y[i] = state.pose.getY();
            Rotation2d desired = state.pose.getRotation();
            headings[i] = i == 0 ? desired : approach(headings[i - 1], desired, maxTurnRate * (time[i] - time[i - 1]));
        }
        for (int i = n - 2; i >= 0; i--) {
            headings[i] = approach(headings[i + 1], headings[i], maxTurnRate * (time[i + 1] - time[i]));
        }
        double[] heading = new double[n];
        for (int i = 0; i < n; i++) {
            heading[i] = headings[i].getRadians();
        }
        return new PathReference(time, x, y, heading, speed);
    }

    private static Rotation2d approach(Rotation2d from, Rotation2d to, double maxStep) {
        return from.plus(Rotation2d.fromRadians(MathUtil.clamp(to.minus(from).getRadians(), -maxStep, maxStep)));
    }

    public double totalTime() {
        return time[time.length - 1];
    }

    public double totalDistance() {
        return distance[distance.length - 1];
    }

    public Pose2d end() {
        int last = time.length - 1;
        return new Pose2d(x[last], y[last], Rotation2d.fromRadians(heading[last]));
    }

    public Sample atTime(double seconds) {
        int i = segment(time, seconds);
        int j = Math.min(i + 1, time.length - 1);
        double span = time[j] - time[i];
        double f = span > 0.0 ? MathUtil.clamp((seconds - time[i]) / span, 0.0, 1.0) : 0.0;
        double turn = MathUtil.angleModulus(heading[j] - heading[i]);
        Pose2d pose = new Pose2d(lerp(x[i], x[j], f), lerp(y[i], y[j], f), Rotation2d.fromRadians(heading[i] + turn * f));
        return new Sample(pose, lerp(distance[i], distance[j], f), lerp(speed[i], speed[j], f), span > 0.0 ? turn / span : 0.0);
    }

    public double timeAtDistance(double meters) {
        int i = segment(distance, meters);
        int j = Math.min(i + 1, time.length - 1);
        double span = distance[j] - distance[i];
        double f = span > 0.0 ? MathUtil.clamp((meters - distance[i]) / span, 0.0, 1.0) : 0.0;
        return lerp(time[i], time[j], f);
    }

    public Projection project(Translation2d point, int fromIndex) {
        int last = time.length - 1;
        int start = fromIndex;
        while (start > 0 && distance[fromIndex] - distance[start - 1] <= kSearchBackMeters) {
            start--;
        }
        Projection best = null;
        double bestGap = Double.MAX_VALUE;
        for (int i = start; i < last && distance[i] <= distance[fromIndex] + kSearchAheadMeters; i++) {
            double dx = x[i + 1] - x[i];
            double dy = y[i + 1] - y[i];
            double length = Math.hypot(dx, dy);
            if (length < kMinSegmentMeters) {
                continue;
            }
            Translation2d tangent = new Translation2d(dx / length, dy / length);
            double along = (point.getX() - x[i]) * tangent.getX() + (point.getY() - y[i]) * tangent.getY();
            double clamped = MathUtil.clamp(along, i == 0 ? Double.NEGATIVE_INFINITY : 0.0, i + 1 == last ? Double.POSITIVE_INFINITY : length);
            double cross = tangent.getX() * (point.getY() - y[i]) - tangent.getY() * (point.getX() - x[i]);
            double gap = Math.hypot(along - clamped, cross);
            if (gap < bestGap) {
                bestGap = gap;
                best = new Projection(i, distance[i] + clamped, cross, tangent);
            }
        }
        if (best == null) {
            int i = Math.max(0, Math.min(fromIndex, last - 1));
            Translation2d tangent = new Translation2d(Math.cos(heading[i]), Math.sin(heading[i]));
            return new Projection(i, distance[i], 0.0, tangent);
        }
        return best;
    }

    public Pose2d[] poses(int count) {
        int step = Math.max(1, time.length / Math.max(1, count));
        return IntStream.iterate(0, i -> i < time.length, i -> i + step)
                .mapToObj(i -> new Pose2d(x[i], y[i], Rotation2d.fromRadians(heading[i])))
                .toArray(Pose2d[]::new);
    }

    private static int segment(double[] values, double value) {
        int i = Arrays.binarySearch(values, value);
        int index = i >= 0 ? i : -i - 2;
        return MathUtil.clamp(index, 0, Math.max(0, values.length - 2));
    }

    private static double lerp(double a, double b, double f) {
        return a + (b - a) * f;
    }
}
