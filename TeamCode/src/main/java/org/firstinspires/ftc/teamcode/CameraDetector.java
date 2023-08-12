package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;

import com.vuforia.TrackableResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CameraDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    static final Rect LEFT_ROI = new Rect(
            new Point(60,35),
            new Point(120,75)
    );
    static final Rect RIGHT_ROI = new Rect(
            new Point(140,35),
            new Point(200,75)
    );
    public CameraDetector(Telemetry t){
        telemetry = t;
    }
    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23,50,70);
        Scalar highHSV = new Scalar(32,255,255);

        Core.inRange(mat,lowHSV,highHSV,mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() /255;
        double rightValue = Core.sumElems(right).val[]
    }
}
