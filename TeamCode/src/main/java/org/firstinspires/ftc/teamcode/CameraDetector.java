package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;

import com.vuforia.TrackableResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CameraDetector extends OpenCvPipeline {
    Telemetry telemetry;

    //insert target purple and target green as well
    static double[] targetOrange = {222, 156, 2};
    static double[] targetWhite = {240, 239, 225};
    static double pctColorError = 0.20;

    //the final value after comparison for which sleeve color is shown
    public static int ballColor;

    //the dimensions of the area that is scanned
    static int scanWidth = 80;
    static int scanHeight = 80;

    //how much the scanned area is translated
    static int xDev = 0;
    static int yDev = 0;

    //brightness increases the color threshold. If it is not detecting, try increasing brightness.
    int brightness = 0;

    public Mat processFrame(Mat input){
        Size imageSize = input.size();
        Mat output = input.clone();
        int orangeCnt = 0;
        int whiteCnt = 0;
        int purpleCnt = 0;
//        Rect crosshair1 = new Rect(new Point(, ), new Point(, );
//        Imgproc.rectangle(output, crosshair1, new Scalar(4,233,78),3,8);
        for(int i = (int)(imageSize.height)/3/2 - scanHeight; i < imageSize.height/3/2 + scanHeight ; i++){
            for(int j = (int)(imageSize.width/3); j < imageSize.width/3 + scanWidth; j++)
            {
                double[] pixelColor = input.get(i,j);
                if (compareColor(targetOrange, pixelColor)){
                    orangeCnt++;
                    double[] newColor = {250, 0, 0, pixelColor[3]};
                    output.put(i, j, newColor);
                }
                if (compareColor(targetWhite, pixelColor)){
                    whiteCnt++;
                    double[] newColor = {0, 250, 0, pixelColor[3]};
                    output.put(i, j, newColor);
                }
            }
        }
        if(orangeCnt > whiteCnt){
            ballColor = 3; //ballColor = 3 means that we park in the orange zone
        }
        if(whiteCnt > orangeCnt){
            ballColor = 1; //ballColor = 1 means that we park in the white zone 
        }

        return output;
    }

    private static boolean compareColor(double[] targetColor, double[] pixelColor) {
        boolean output = true;
        for(int i = 0; i < 3; i++) {
            output = output && pixelColor[i] < targetColor[i] * (1 + pctColorError) && pixelColor[i] > targetColor[i] * (1 - pctColorError);
        }
        return output;
    }
}
