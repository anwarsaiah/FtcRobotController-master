/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear Opmode")
//@Disabled
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx rotate = null;
    private DcMotor arm = null;
    private DcMotor lift = null;
    private AnalogInput liftPosition = null;
    private RevTouchSensor magnet = null;
    private PIDController pid =null, rotatePID=null;
    private Servo grab = null, wrist = null;
    private TouchSensor armShort;
    private TouchSensor armLong;
    double liftHoldPosition = 1.0;//0.989;
    int rotateHoldPosition = 0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "exM0");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "exM1");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "exM2");
        rightBackDrive = hardwareMap.get(DcMotor.class, "exM3");
        rotate = hardwareMap.get(DcMotorEx.class, "rotate");
        arm = hardwareMap.get(DcMotor.class,"arm");
        lift = hardwareMap.get(DcMotor.class,"lift");
        liftPosition = hardwareMap.get(AnalogInput.class,"liftPosition");
        magnet = hardwareMap.get(RevTouchSensor.class, "magnet");
        grab = hardwareMap.get(Servo.class,"ser1");
        wrist = hardwareMap.get(Servo.class,"ser2");
        armShort = hardwareMap.get(TouchSensor.class,"armShort");
        armLong = hardwareMap.get(TouchSensor.class,"armLong");
        pid = new PIDController(3, 0, 0);
        rotatePID = new PIDController(0.05, 0.0, 0.01);

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        ///////////////////////////
        waitForStart();
        //////////////////////////
        wrist.setPosition(0);
        grab.setPosition(0.5);
        runtime.reset();
        int armPosition = arm.getCurrentPosition();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.a)
                grab.setPosition(0);
            if(gamepad1.b)
                grab.setPosition(0.5);
            if(gamepad1.x)
                wrist.setPosition(wrist.getPosition()+0.05);
            if(gamepad1.y)
                wrist.setPosition(wrist.getPosition()-0.05);
            ////wrist control../////////////
            wristControl();
            /////lift control../////////////
            liftControl();
            ////rotate control..////////////
            rotateControl();
            //////arm  control..////////////
            if(gamepad1.right_trigger>0 || gamepad1.left_trigger>0)
            {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                 if(!armShort.isPressed()&&!armLong.isPressed())
                       arm.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
                 else
                       arm.setPower(0);
                 armPosition = arm.getCurrentPosition();
            }
            else {
                arm.setTargetPosition(armPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.4);
            }
            ////mecanum drive ..///////////
            driveMecanum();
            telemetry.addData("armShort", armShort.isPressed());
            telemetry.addData("armLong", armLong.isPressed());
            telemetry.addData("magnet", magnet.getValue());
            telemetry.addData("lift position", liftPosition.getVoltage());
            telemetry.addData("ser1", grab.getPosition());
            telemetry.addData("ser2", wrist.getPosition());
            telemetry.addData("Rotate position", rotate.getCurrentPosition());
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();
        }
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void wristControl(){
        //////wrist position
        double wristPosition = (liftPosition.getVoltage()-0.3)*0.7;
        if(wristPosition<0)wristPosition = 0;
        if(wristPosition>1)
            wristPosition=1;
        if(gamepad1.dpad_up || gamepad1.dpad_down)
             wrist.setPosition(wristPosition);
        //////
    }

    public void liftControl(){
        /////lift control../////////////
        if(gamepad1.dpad_up)
        {
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setPower(0.6);
            liftHoldPosition = liftPosition.getVoltage()+0.08;
        }

        else if (gamepad1.dpad_down) {
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setPower(-0.6);
            liftHoldPosition = liftPosition.getVoltage();
        }
        else
        {
            pid.rest = liftHoldPosition;
            pid.input = liftPosition.getVoltage();
            pid.calculate();
            lift.setPower(pid.output);
        }
    }
    public void rotateControl(){
        if(gamepad1.dpad_left)
        {
            rotate.setPower(0.9);
            rotateHoldPosition = rotate.getCurrentPosition();
        }
        else if (gamepad1.dpad_right) {
            {
                rotate.setPower(-0.9);
                rotateHoldPosition = rotate.getCurrentPosition();
            }
        }
        else {
            if(rotateHoldPosition!=0){
            rotatePID.rest= rotateHoldPosition;
            rotatePID.input = rotate.getCurrentPosition();
            rotatePID.calculate();
            rotate.setPower(rotatePID.output);
            }
        }
    }
    public void driveMecanum(){
        ///////drive wheels..Mecanum
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
    }
}
