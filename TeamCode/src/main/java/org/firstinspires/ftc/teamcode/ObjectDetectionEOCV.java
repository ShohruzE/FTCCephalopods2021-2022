package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ObjectDetectionEOCV extends OpenCvPipeline {

    Mat HSVmat = new Mat();
    Mat contoursOnFrameMat = new Mat();

    List<MatOfPoint> contoursList = new ArrayList<>();
    int numContoursFound = 0;

    // Define HSV threshold
    public Scalar lowerHSV = new Scalar(19,89,172);
    public Scalar upperHSV = new Scalar(59,250,250);

    public double yThreshold = 50;
    public double blurConstant = 1;
    public double dilationConstant = 2;

    int duckPosition = 0;

    Telemetry telemetryOpenCV;

    public ObjectDetectionEOCV(Telemetry t) {
        telemetryOpenCV = t;
    }

    @Override
    public Mat processFrame(Mat input) {

        // Clears any existing contours, then converts image from RGB to HSV(gray-scale)
        contoursList.clear();
        Imgproc.cvtColor(input, HSVmat, Imgproc.COLOR_RGB2HSV_FULL);

        // Filters out all colors but yellow
        Core.inRange(HSVmat, lowerHSV, upperHSV, HSVmat);

        Size kernelSize = new Size(blurConstant, blurConstant);

        // Blurs the image
        Imgproc.GaussianBlur(HSVmat, HSVmat, kernelSize, 0);

        Size kernelSize2 =  new  Size(2 * dilationConstant + 1,
                2 * dilationConstant + 1);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelSize2);

        Imgproc.dilate(HSVmat, HSVmat, kernel);

        // Finds contours of the duck
        Imgproc.findContours(HSVmat, contoursList, new Mat(), Imgproc.RETR_TREE,
                Imgproc.CHAIN_APPROX_SIMPLE);

        numContoursFound = contoursList.size();
        input.copyTo(contoursOnFrameMat);

        for(MatOfPoint contour : contoursList){
            Rect rect = Imgproc.boundingRect(contour);

            if (rect.y >= yThreshold){
                Imgproc.rectangle(contoursOnFrameMat, rect.tl(), rect.br(), new Scalar(255, 0, 0), 2);
                Imgproc.putText(contoursOnFrameMat, String.valueOf(rect.x), rect.tl(), 0, 0.5,
                                                                                    new Scalar(255, 255, 255));

                if (rect.x <= 100) {
                    duckPosition = 0;
                }
                else if (rect.x >= 190) {
                    duckPosition = 2;
                }
                else {
                    duckPosition = 1;
                }
            }
        }

        return input;
    }

    public int getDuckPosition() {
        return duckPosition;
    }
}
