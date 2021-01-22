package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "AutoBot2")
public class UpdateRobotPositionAuto extends LinearRobot {



    @Override
    public void runOpMode() throws InterruptedException {

        super.runOpMode();

        waitForStart();

        positionUpdate = new OdometryGlobalCoordinateSystem(leftEncoder, rightEncoder, middleEncoder, TICKS_PER_INCH, 100);
        Thread position = new Thread(positionUpdate);
        position.start();

        while (opModeIsActive()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            if (pipeline.position.equals(EasyOpenCvWebcam.UltimateGoalPipeline.ringPosition.FOUR)){

            } else if (pipeline.position.equals(EasyOpenCvWebcam.UltimateGoalPipeline.ringPosition.ONE)){

            } else {

            }
            robotPlane(0 * TICKS_PER_INCH, 24 * TICKS_PER_INCH, .5, 0 *TICKS_PER_INCH, 1);
        }
    }
}
