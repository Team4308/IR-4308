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

        File file = new File(Filesystem.getDeployDirectory().getAbsolutePath() + profileFileName + ".path");

        try (BufferedReader csvReader = new BufferedReader(new FileReader(file))) {
            ArrayList<double[]> pointArray = new ArrayList<double[]>();

            String line;
            while ((line = csvReader.readLine()) != null) {
                double[] point = Arrays.stream(line.split(",")).mapToDouble(Double::parseDouble).toArray();
                pointArray.add(point);
            }


        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void initBuffer() {

    }
}