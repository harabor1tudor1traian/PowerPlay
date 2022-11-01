package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class ElementDetection extends OpenCvPipeline {
    private OpenCvCamera camera;
    private Mat imageHSV = new Mat();
    private Mat imageHSV2 = new Mat();
    private Mat pic1 = new Mat();
    private Mat pic2 = new Mat();
    private Mat pic3 = new Mat();
    public static int randomisation = 0;
    static final Rect screen = new Rect(
            new Point( 400, 0 ),
            new Point( 900, 720 ) );

    private Telemetry telemetry;
    public ElementDetection(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry=telemetry;
        int camMonViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName()
        );
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                camMonViewId
        );
        camera.setPipeline(this);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener( ) {
                                         @Override
                                         public void onOpened( ) {
                                             camera.startStreaming( 1280, 720, OpenCvCameraRotation.UPRIGHT);
                                         }

                                         @Override
                                         public void onError( int errorCode ) {
                                             //This will be called if the camera could not be opened
                                             Log.e( "CAMERA_DEVICE", "Camera could not be opened. Error code: " + errorCode );
                                         }
                                     }
        );

    }
    @Override
    public Mat processFrame(Mat input){
        //Convert input to HSV
        Imgproc.cvtColor(input,imageHSV,Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(imageHSV,imageHSV,Imgproc.COLOR_RGB2HSV);

        //Color Thresholds
        Scalar lowerBound1 = new Scalar(36,39,87);
        Scalar upperBound1 = new Scalar(88,105,159);

        Scalar lowerBound2 = new Scalar(24,52,51);
        Scalar upperBound2 = new Scalar(81,166,146);

        Scalar lowerBound3 = new Scalar(116,68,67);
        Scalar upperBound3 = new Scalar(149,118,173);
        
        Core.inRange(imageHSV,lowerBound1,upperBound1,imageHSV2);
        pic1 = imageHSV2;
        Core.inRange(imageHSV,lowerBound2,upperBound2,imageHSV2);
        pic2 = imageHSV2;
        Core.inRange(imageHSV,lowerBound3,upperBound3,imageHSV2);
        pic3 = imageHSV2;


        //Get Color Average
        double value1 = Core.mean(pic1).val[0];
        double value2 = Core.mean(pic2).val[0];
        double value3 = Core.mean(pic3).val[0];


        //Clear Memory
        pic1.release();
        pic2.release();
        pic3.release();
        imageHSV2.release();

        //Compare Values
        double max = Math.max(value1, Math.max(value3, value2));

        if(value1 == max){
            randomisation = 1;
        }
        else if(value2 == max){
            randomisation = 2;

        }
        else if(value3 == max){
            randomisation = 3;

        }
        else{
            randomisation = 2;
        }
        telemetry.addLine("DONE");
        telemetry.addData("1: ", value1);
        telemetry.addData("2: ", value2);
        telemetry.addData("3: ", value3);
        telemetry.addData("case: ", randomisation);
        telemetry.update();
        return input;
    }
}
