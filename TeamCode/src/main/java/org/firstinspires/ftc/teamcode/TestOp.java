package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "TestOpOnly")
public class TestOp extends LinearRobot{

    @Override
    public void runOpMode() throws InterruptedException {

        super.runOpMode();

        waitForStart();

        while (opModeIsActive()) {

            drive();

            if (gamepad1.b) {
                shooterPower(1);
            }

            if (gamepad1.a){
                kicker();
            }

            intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            telemetry.addData("WobbleArmPosition", wobbleArm.getCurrentPosition());
            telemetry.update();
        }
    }
}
