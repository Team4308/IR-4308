package frc.robot.motionprofile;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;

public class MotionStream {
    private final BufferedTrajectoryPointStream internal_stream;

    public MotionStream(String profileFileName) {
        internal_stream = new BufferedTrajectoryPointStream();

        File file = new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/" + profileFileName + ".csv");

        try (BufferedReader csvReader = new BufferedReader(new FileReader(file))) {
            ArrayList<double[]> pointArray = new ArrayList<double[]>();

            String line;
            while ((line = csvReader.readLine()) != null) {
                double[] point = Arrays.stream(line.split(",")).mapToDouble(Double::parseDouble).toArray();
                pointArray.add(point);
            }

            double[][] doubleArray = new double[3][pointArray.size()];
            pointArray.toArray(doubleArray);

            initBuffer(doubleArray);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void initBuffer(double[][] points) {
        int length = points.length;

        TrajectoryPoint tpoint = new TrajectoryPoint();

        for (int i = 0; i < length; i++) {
            double position = points[0][i];
            double velocity = points[1][i];
            int duration = (int)points[2][i];

            tpoint.timeDur = duration;
            tpoint.position = position * Constants.Config.Drive.Kinematics.kSensorUnitsPerRotation;
            tpoint.velocity = velocity * Constants.Config.Drive.Kinematics.kSensorUnitsPerRotation / 600.0;

            tpoint.auxiliaryPos = 0.0;
            tpoint.auxiliaryVel = 0.0;
            tpoint.profileSlotSelect0 = Constants.Config.Drive.MotionProfile.profileSlot;
            tpoint.profileSlotSelect1 = 0;
            tpoint.zeroPos = (i == 0);
            tpoint.isLastPoint = ((i + 1) == length);
            tpoint.arbFeedFwd = 0.0;

            internal_stream.Write(tpoint);
        }
    }

    public BufferedTrajectoryPointStream getStream() {
        return internal_stream;
    }
}