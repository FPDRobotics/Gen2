package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@Autonomous
public class OdometryCalibration extends LinearRobot {

    DcMotor fl, fr, bl, br;
    DcMotor leftEncoder, rightEncoder, middleEncoder;

    BNO055IMU imu;

    ElapsedTime timer = new ElapsedTime();

    //sets power of motor
    static final double calibrationSpeed = .5;

    //going to change after known type of drivetrain motor in detail *ask Jack or Justin*

    static final double TICKS_PER_REV = 537.6;

    static final double WHEEL_DIAMETER = 100/25.4;

    //Next, is the gear ratio needs to also be changed once drivetrain motors' info is found

    static final double GEAR_RATIO = 1;

    static final double TICKS_PER_INCH = WHEEL_DIAMETER * Math.PI * GEAR_RATIO / TICKS_PER_REV;

    File sidewheelSeparationFile = AppUtil.getInstance().getSettingsFile("sidewheelsSeperationFile");
    File middleTickOffsetFile  = AppUtil.getInstance().getSettingsFile("middleTickOffsetFile");

    @Override
    public void runOpMode(){
        leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
        rightEncoder = hardwareMap.dcMotor.get("rightEncoder");
        middleEncoder = hardwareMap.dcMotor.get("middleEncoder");

        resetOdometryEncoder();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status:", "Ready!");
        telemetry.update();

        waitForStart();

        while (imu.getAngularOrientation().thirdAngle < 90 && opModeIsActive()){
            fr.setPower(-calibrationSpeed);
            fl.setPower(calibrationSpeed);
            bl.setPower(calibrationSpeed);
            br.setPower(-calibrationSpeed);
            if(imu.getAngularOrientation().thirdAngle < 60){
                fr.setPower(-calibrationSpeed);
                fl.setPower(calibrationSpeed);
                bl.setPower(calibrationSpeed);
                br.setPower(-calibrationSpeed);
            }else {
                fr.setPower(-calibrationSpeed / 2);
                fl.setPower(calibrationSpeed / 2);
                bl.setPower(calibrationSpeed / 2);
                br.setPower(-calibrationSpeed / 2);
            }
        }
        fr.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        timer.reset();
        while(timer.seconds() < 1 && opModeIsActive()){
            telemetry.addLine("waiting....");
        }

        double angle = imu.getAngularOrientation().thirdAngle;
        double encoderDifference = Math.abs(Math.abs(leftEncoder.getCurrentPosition()) - Math.abs(rightEncoder.getCurrentPosition()) - Math.abs(middleEncoder.getCurrentPosition()));
        double sideEncoderTicksOffset = encoderDifference / angle;
        double sideWheelSeparation = (180 * sideEncoderTicksOffset ) / (TICKS_PER_INCH * Math.PI);
        double middleTickOffset = middleEncoder.getCurrentPosition() / Math.toRadians(imu.getAngularOrientation().thirdAngle);

        ReadWriteFile.writeFile(sidewheelSeparationFile, String.valueOf(sideWheelSeparation));
        ReadWriteFile.writeFile(middleTickOffsetFile, String.valueOf(middleTickOffset));
    }

    void resetOdometryEncoder(){
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
