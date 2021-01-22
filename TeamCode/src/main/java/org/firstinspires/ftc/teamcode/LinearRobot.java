package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

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
import org.openftc.easyopencv.OpenCvWebcam;


public class LinearRobot extends LinearOpMode {

    public DcMotor fl, fr, bl, br;
    public DcMotor shooter1, shooter2;
    public DcMotor wobbleArm, intake;
    public Servo kicker, claw;
    DcMotor leftEncoder, rightEncoder, middleEncoder;
    public BNO055IMU imu;
    //final private int lowerWobbleArm = ________;
    //final private int raiseWobbleArm = _______;
    //final private int backToZeroPosition = _____;
    public OpenCvWebcam Webcam1;
    public EasyOpenCvWebcam.UltimateGoalPipeline pipeline;

    static final double TICKS_PER_REV = 537.6;
    static final double WHEEL_DIAMETER = 100/25.4;
    static final double GEAR_RATIO = 1;

    static final double TICKS_PER_INCH = WHEEL_DIAMETER * Math.PI * GEAR_RATIO / TICKS_PER_REV;

    OdometryGlobalCoordinateSystem positionUpdate;

    public void runOpMode() throws InterruptedException {

        intake = hardwareMap.dcMotor.get(RobotNames.intakeMotor);
        fl = hardwareMap.dcMotor.get(RobotNames.frontLeftMotor);
        fr = hardwareMap.dcMotor.get(RobotNames.frontRightMotor);
        bl = hardwareMap.dcMotor.get(RobotNames.backLeftMotor);
        br = hardwareMap.dcMotor.get(RobotNames.backRightMotor);
        shooter1 = hardwareMap.dcMotor.get(RobotNames.shooter1Motor);
        shooter2 = hardwareMap.dcMotor.get(RobotNames.shooter2Motor);
        wobbleArm = hardwareMap.dcMotor.get(RobotNames.wobbleArmMotor);
        kicker = hardwareMap.servo.get(RobotNames.kickerServo);
        claw = hardwareMap.servo.get(RobotNames.wobbleClawServo);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        pipeline = new EasyOpenCvWebcam.UltimateGoalPipeline();
        Webcam1.setPipeline(pipeline);

        Webcam1.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        Webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened(){
                Webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.addData("FPS", String.format("%.2f", Webcam1.getFps()));
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        resetOdometryEncoders();

        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setZeroPowerDrivetrain(DcMotor.ZeroPowerBehavior.BRAKE);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        wobbleArm.setDirection( DcMotorSimple.Direction.REVERSE);

    }

    public void drive() {

        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (Math.PI / 4);
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        if(gamepad1.left_stick_button) {
            fl.setPower(v1 * .7);
            fr.setPower(v2 * .7);
            bl.setPower(v3 * .7);
            br.setPower(v4 * .7);
        }else {
            fl.setPower(v1 / 2);
            fr.setPower(v2 / 2);
            bl.setPower(v3 / 2);
            br.setPower(v4 / 2);
        }
    }

    public void kicker() {
        kicker.setPosition(1);
        sleep(100);
        kicker.setPosition(0);
    }

    public void ComputeMotorPowers(double vx, double vy, double a, long t){
        final double r = 2;
        final double lx = 12.776;
        final double ly = 14.258;

        double w1 = (1 / r) * (vx - vy - (lx + ly) * a);
        double w2 = (1 / r) * (vx + vy + (lx + ly) * a);
        double w3 = (1 / r) * (vx + vy - (lx + ly) * a);
        double w4 = (1 / r) * (vx - vy + (lx + ly) * a);

        fl.setPower(w1);
        fr.setPower(w2);
        bl.setPower(w3);
        br.setPower(w4);

        sleep(t);
    }

    public void setZeroPowerDrivetrain (DcMotor.ZeroPowerBehavior behavior){
        fl.setZeroPowerBehavior(behavior);
        fr.setZeroPowerBehavior(behavior);
        bl.setZeroPowerBehavior(behavior);
        br.setZeroPowerBehavior(behavior);
    }

    public void shooterPower(double pwr){
        shooter1.setPower(pwr);
        shooter2.setPower(pwr);
    }

    public static class UltimateGoalPipeline extends OpenCvPipeline {

        public enum ringPosition{
            FOUR,
            ONE,
            NONE
        }

        static final Scalar BLUE = new Scalar(0, 0, 225);
        static final Scalar GREEN = new Scalar(0, 225, 0);

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(50, 200);

        static final int REGION_WIDTH = 45;
        static final int REGION_HEIGHT = 35;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);

        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avgl;

        public ringPosition position = ringPosition.FOUR;
        public ringPosition count = ringPosition.FOUR;

        void inputToCb(Mat input){
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame){
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avgl = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    BLUE,
                    2);

            position = ringPosition.FOUR;
            count = ringPosition.FOUR;

            if (avgl > FOUR_RING_THRESHOLD) {
                position = ringPosition.FOUR;
            } else if (avgl > ONE_RING_THRESHOLD) {
                position = ringPosition.ONE;
            } else {
                position = ringPosition.NONE;
            }

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    GREEN,
                    -1);

            return input;
        }
        public int getAnalysis() {
            return avgl;
        }
    }

    void resetOdometryEncoders(){
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void robotPlane(double Xposition, double Yposition, double robotPwr, double robotAngle, double error){
        double X_distance = Xposition - positionUpdate.returnXCoordinate();
        double Y_distance = Yposition - positionUpdate.returnYCoordinate();

        double distance = Math.hypot(X_distance, Y_distance);

        while (opModeIsActive() && distance > error) {
            distance = Math.hypot(X_distance, Y_distance);
            X_distance = Xposition - positionUpdate.returnXCoordinate();
            Y_distance = Yposition - positionUpdate.returnYCoordinate();

            double robotMovementAngle = Math.toDegrees(Math.atan2(X_distance, Y_distance));

            double X_component = calculateX(robotMovementAngle, robotPwr);
            double Y_component = calculateY(robotMovementAngle, robotPwr);
            double offCorrection = robotAngle - positionUpdate.returnOrientation();
        }
    }

    private double calculateX(double desiredAngle, double speed){
        return Math.sin(Math.toRadians(desiredAngle) * speed);
    }

    private double calculateY(double desiredAngle, double speed){
        return Math.cos(Math.toRadians(desiredAngle) * speed);
    }

    //public void raiseWobbleArm() {
        //wobbleArm.setTargetPosition(raiseWobbleArm);
        //wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //wobbleArm.setPower(.2);
    //}

    //public void lowerWobbleArm() {
        //wobbleArm.setTargetPosition(lowerWobbleArm);
        //wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //wobbleArm.setPower(.2);
    //}

    //public void startingPosition() {
        //wobbleArm.setTargetPosition(backToZeroPosition);
        //wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //wobbleArm.setPower(.2);
    //}

}
