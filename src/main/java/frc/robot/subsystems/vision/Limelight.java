package frc.robot.util;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Subsystem for interacting with the Limelight 2
 */
public class Limelight extends Subsystem {
    
    private final NetworkTable mNetworkTable;
    private boolean mOutputsHaveChanged = true;

    private final PeriodicIO mPeriodicIO = new PeriodicIO();
    private static final double[] kEmptyDoubleArray = new double[0];
    private int mListenerId = -1;

    private static Limelight mInstance;

    public static Limelight getInstance() {
        if (mInstance == null) {
            mInstance = new Limelight();
        }
        return mInstance;
    }
}