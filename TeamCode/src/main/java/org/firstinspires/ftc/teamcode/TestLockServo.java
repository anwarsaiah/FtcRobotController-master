package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Angle Rotation", group = "Robot")
public class TestLockServo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        robot.init();

        waitForStart();
        robot.lock.setPosition(0.5);
        while (opModeIsActive()){


        }
    }
}
