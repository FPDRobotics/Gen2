package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class UpdateRobotPositionTele extends LinearRobot {

    @Override
    public void runOpMode(){
        resetOdometryEncoders();

        waitForStart();

        positionUpdate = new OdometryGlobalCoordinateSystem(leftEncoder, rightEncoder, middleEncoder, TICKS_PER_INCH, 100);
        Thread position = new Thread(positionUpdate);
        position.start();

        while (opModeIsActive()) {
            drive();

            telemetry.addData("X position", positionUpdate.returnXCoordinate() / TICKS_PER_INCH);
            telemetry.addData("Y position", positionUpdate.returnYCoordinate() / TICKS_PER_INCH);
            telemetry.addData("Orientation (degree)", positionUpdate.returnOrientation());
            telemetry.update();
        }
        positionUpdate.stop();
    }


}
