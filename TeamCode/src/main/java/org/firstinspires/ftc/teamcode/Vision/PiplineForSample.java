package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class PiplineForSample implements VisionProcessor {

    // Image processing variables
    private Mat ycrcbMat = new Mat();
    private Mat cbMat = new Mat();
    private Mat yellowThresholdMat = new Mat();
    private Mat morphedYellowThreshold = new Mat();

    // Constants for threshold and morphological operations
    private static final int YELLOW_MASK_THRESHOLD = 57;
    private Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));
    private Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));

    // Scalar for yellow color visualization
    private static final Scalar YELLOW = new Scalar(255, 255, 0);

    // Detected sample details
    private Point yellowSampleCenter = new Point(0, 0);
    private double yellowSampleAngle = 0.0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Initialize resources if needed (e.g., pre-allocate Mats)
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert the input image to YCrCb color space
        Imgproc.cvtColor(frame, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

        // Extract the Cb channel
        Core.extractChannel(ycrcbMat, cbMat, 2);

        // Apply a binary threshold to isolate yellow
        Imgproc.threshold(cbMat, yellowThresholdMat, YELLOW_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);

        // Apply morphological operations for noise reduction
        morphMask(yellowThresholdMat, morphedYellowThreshold);

        // Find contours for the yellow regions
        ArrayList<MatOfPoint> yellowContoursList = new ArrayList<>();
        Imgproc.findContours(morphedYellowThreshold, yellowContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // Analyze the largest contour (if any)
        if (!yellowContoursList.isEmpty()) {
            MatOfPoint largestContour = findLargestContour(yellowContoursList);
            analyzeContour(largestContour, frame);
        }

        // Return data or state for external use if needed
        return new YellowSampleData(yellowSampleCenter, yellowSampleAngle);
    }

    private void morphMask(Mat input, Mat output) {
        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);
        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    private MatOfPoint findLargestContour(ArrayList<MatOfPoint> contours) {
        MatOfPoint largestContour = null;
        double maxArea = 0;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        return largestContour;
    }

    private void analyzeContour(MatOfPoint contour, Mat input) {
        // Transform the contour to a different format
        Point[] points = contour.toArray();
        MatOfPoint2f contour2f = new MatOfPoint2f(points);

        // Fit a rotated rectangle to the contour
        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);

        // Calculate the angle
        double rotRectAngle = rotatedRectFitToContour.angle;
        if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height) {
            rotRectAngle += 90;
        }

        yellowSampleAngle = -(rotRectAngle - 180);
        yellowSampleCenter = rotatedRectFitToContour.center;

        // Draw the rotated rectangle and angle on the frame
        drawRotatedRect(rotatedRectFitToContour, input);
    }

    private void drawRotatedRect(RotatedRect rect, Mat drawOn) {
        Point[] points = new Point[4];
        rect.points(points);

        for (int i = 0; i < 4; ++i) {
            Imgproc.line(drawOn, points[i], points[(i + 1) % 4], YELLOW, 2);
        }

        Imgproc.putText(
                drawOn,
                String.format("%.1f deg", yellowSampleAngle),
                new Point(rect.center.x - 50, rect.center.y + 25),
                Imgproc.FONT_HERSHEY_PLAIN,
                1,
                YELLOW,
                1);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Implement any necessary drawing on the canvas (e.g., overlay data visualization)
    }

    public static class YellowSampleData {
        public final Point center;
        public final double angle;

        public YellowSampleData(Point center, double angle) {
            this.center = center;
            this.angle = angle;
        }
    }
}
