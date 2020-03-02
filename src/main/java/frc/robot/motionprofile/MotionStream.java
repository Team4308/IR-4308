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
    public BufferedTrajectoryPointStream internal_stream;

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

            double[][] doubleArray = new double[pointArray.size()][3];
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
            //System.out.println("Pos: " + points[i][0] + " | Vel: " + points[i][1] + " | Dur: " + points[i][2]);
            double position = points[i][0];
            double velocity = points[i][1];
            int duration = (int)points[i][2];

            tpoint.timeDur = duration;
            tpoint.position = (-1 * position / Constants.Config.Drive.Kinematics.kGearRatio) * (Constants.Config.Drive.Kinematics.kSensorUnitsPerRotation);
            tpoint.velocity = ((-1 * velocity) * Constants.Config.Drive.Kinematics.kSensorUnitsPerRotation);

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
}