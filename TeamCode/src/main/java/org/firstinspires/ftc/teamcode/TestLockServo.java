package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Lock", group = "Robot")
public class TestLockServo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        robot.init();

        waitForStart();
        robot.lock.setPosition(0.5);
        while (opModeIsActive()){
            if(gamepad1.a){
                robot.lock.setPosition(robot.lock.getPosition()+0.01);
                sleep(100);  //0.4 locked    0.55 opened
            }
            if(gamepad1.b){
                robot.lock.setPosition(robot.lock.getPosition()-0.01);
                sleep(100);
            }

            telemetry.addData("Lock", robot.lock.getPosition());
            telemetry.update();
        }
    }
}
