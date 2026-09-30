package frc.robot.subsystems.vision;

import edu.wpi.first.util.struct.Struct;
import frc.robot.util.LimelightHelpers;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Objects;

public record FiducialObservation(int id, double txnc, double tync, double ambiguity, double area)
        {
    public static FiducialObservation fromLimelight(LimelightHelpers.RawFiducial fiducial) {
        if (fiducial == null) {
            return null;
        }
        return new FiducialObservation(
                fiducial.id, fiducial.txnc, fiducial.tync, fiducial.ambiguity, fiducial.ta);
    }

    public static FiducialObservation[] fromLimelight(LimelightHelpers.RawFiducial[] fiducials) {
        if (fiducials == null) {
            return new FiducialObservation[0];
        }
        return Arrays.stream(fiducials)
                .map(FiducialObservation::fromLimelight)
                .filter(Objects::nonNull)
                .toArray(FiducialObservation[]::new);
    }

    public static final Struct<FiducialObservation> struct =
            new Struct<FiducialObservation>() {
                @Override
                public Class<FiducialObservation> getTypeClass() {
                    return FiducialObservation.class;
                }

                @Override
                public String getTypeString() {
                    return "record:FiducialObservation";
                }

                @Override
                public int getSize() {
                    return Integer.BYTES + 4 * Double.BYTES;
                }

                @Override
                public String getSchema() {
                    return "int id;double txnc;double tync;double ambiguity";
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
                    bb.putInt(value.id());
                    bb.putDouble(value.txnc());
                    bb.putDouble(value.tync());
                    bb.putDouble(value.ambiguity());
                    bb.putDouble(value.area());
                }

                @Override
                public String getTypeName() {
                    return "FiducialObservation";
                }
            };
}