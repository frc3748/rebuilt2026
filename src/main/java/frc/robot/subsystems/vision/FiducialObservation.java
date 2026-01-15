package frc.robot.subsystems.vision;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.util.LimelightHelpers;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Objects;

/**
 * Represents an observation of a fiducial marker (AprilTag) with position and quality data.
 */
public class FiducialObservation implements StructSerializable {
    public final int id;
    public final double txnc;
    public final double tync;
    public final double ambiguity;
    public final double area;

    public FiducialObservation(int id, double txnc, double tync, double ambiguity, double area) {
        this.id = id;
        this.txnc = txnc;
        this.tync = tync;
        this.ambiguity = ambiguity;
        this.area = area;
    }

    /** Converts a Limelight raw fiducial to a FiducialObservation. */
    public static FiducialObservation fromLimelight(LimelightHelpers.RawFiducial fiducial) {
        if (fiducial == null) return null;
        return new FiducialObservation(
                fiducial.id, fiducial.txnc, fiducial.tync, fiducial.ambiguity, fiducial.ta);
    }

    /** Converts an array of Limelight raw fiducials to FiducialObservation array. */
    public static FiducialObservation[] fromLimelight(LimelightHelpers.RawFiducial[] fiducials) {
        if (fiducials == null) return new FiducialObservation[0];
        return Arrays.stream(fiducials)
                .map(FiducialObservation::fromLimelight)
                .filter(Objects::nonNull)
                .toArray(FiducialObservation[]::new);
    }

    public static final FiducialObservationStruct struct = new FiducialObservationStruct();

    public static class FiducialObservationStruct implements Struct<FiducialObservation> {
        @Override
        public Class<FiducialObservation> getTypeClass() {
            return FiducialObservation.class;
        }

        @Override
        public String getTypeName() {
            return "FiducialObservation";
        }

        @Override
        public String getTypeString() {
            return "struct:FiducialObservation";
        }

        @Override
        public int getSize() {
            return Integer.BYTES + (4 * Double.BYTES);
        }

        @Override
        public String getSchema() {
            return "int32 id;double txnc;double tync;double ambiguity;double area";
        }

        @Override
        public FiducialObservation unpack(ByteBuffer bb) {
            int id = bb.getInt();
            double txnc = bb.getDouble();
            double tync = bb.getDouble();
            double ambiguity = bb.getDouble();
            double area = bb.getDouble();
            return new FiducialObservation(id, txnc, tync, ambiguity, area);
        }

        @Override
        public void pack(ByteBuffer bb, FiducialObservation value) {
            bb.putInt(value.id);
            bb.putDouble(value.txnc);
            bb.putDouble(value.tync);
            bb.putDouble(value.ambiguity);
            bb.putDouble(value.area);
        }
    }
}