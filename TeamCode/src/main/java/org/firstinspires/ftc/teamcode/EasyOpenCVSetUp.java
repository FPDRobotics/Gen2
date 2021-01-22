package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class EasyOpenCVSetUp extends LinearOpMode {

    OpenCvWebcam Webcam1;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);

        Webcam1.setPipeline(new SamplePipeline());

        Webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                Webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Frame Count", Webcam1.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", Webcam1.getFps()));
            telemetry.addData("Total frame time ms", Webcam1.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", Webcam1.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", Webcam1.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", Webcam1.getCurrentPipelineMaxFps());
            telemetry.update();

            if (gamepad1.a){

                Webcam1.stopStreaming();

            }

            sleep(100);
        }
    }

    class SamplePipeline extends OpenCvPipeline {

        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input){

            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            return input;
        }

        @Override
        public void onViewportTapped(){

            viewportPaused = !viewportPaused;

            if(viewportPaused){

                Webcam1.pauseViewport();

            } else{
                Webcam1.resumeViewport();
            }
        }
    }
}
