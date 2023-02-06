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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear Opmode")
//@Disabled
public class BasicOmniOpMode_Linear extends LinearOpMode {
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
    private PIDController liftPID =null, rotatePID=null;
    private Servo wrist = null, claw = null, flipHand = null, lock = null, claw2 = null;
    private Servo ser5 = null;
    private TouchSensor armLong;
    private TouchSensor armShort;
    double liftHoldPosition = 1.0;//0.989;
    int rotateHoldPosition = 0;
    int armPosition, liftEncoder;
    boolean longArm = false, shortArm = false, armReset = false;

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
        wrist = hardwareMap.get(Servo.class,"wrist");
        claw = hardwareMap.get(Servo.class,"claw");
        lock = hardwareMap.get(Servo.class, "lock");
        flipHand = hardwareMap.get(Servo.class, "flip");
        claw2 = hardwareMap.get(Servo.class, "claw2");

        ser5 = hardwareMap.get(Servo.class, "ser5");

        armShort = hardwareMap.get(TouchSensor.class,"armShort");
        armLong = hardwareMap.get(TouchSensor.class,"armLong");
        liftPID = new PIDController(5, 0, 0.05);
        rotatePID = new PIDController(0.035, 0.0, 0.002);


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
        resetArm();
        ///////////////////////////
        waitForStart();
        //////////////////////////
        //claw.setPosition(0.0);
        //claw2.setPosition(1.0);
        //flipHand.setPosition(0.52);  //start --->0.403 end
        //wrist.setPosition(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftEncoder = lift.getCurrentPosition();
        runtime.reset();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armPosition = arm.getCurrentPosition();
        flipHand.setPosition(0.52);//flipped 0.403 --> straight 0.52
        //wrist.setPosition(0);// 0.0 start -->0.4 end

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.a)
            {
                flipHand.setPosition(0.52);
                sleep(100);
            }
            if(gamepad1.b)
            {
                flipHand.setPosition(0.403);
                sleep(100);
            }
            if(gamepad1.x)
            {
                claw.setPosition(0.5);
                claw2.setPosition(0.0); //working
            }
            if(gamepad1.y){
                claw.setPosition(0.0);
                claw2.setPosition(1);
            }
                //flipHand.setPosition(0.405);
            if(gamepad1.right_bumper)
                flipHand.setPosition(0.405);
            if(gamepad1.left_bumper)
                flipHand.setPosition(0.52);
            ////wrist control../////////////
            //wristControl();
            /////lift control../////////////
            liftControl();
            ////rotate control..////////////
            rotateControl();
            //////arm  control..////////////
            armControl();
//            if(gamepad1.right_trigger>0 || gamepad1.left_trigger>0)
//            {
//                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                 if(!armShort.isPressed()&&!armLong.isPressed())
//                       arm.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
//                 else
//                       arm.setPower(0);
//                 armPosition = arm.getCurrentPosition();
//            }
//            else {
//                arm.setTargetPosition(armPosition);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                arm.setPower(0.4);
//            }
            ////mecanum drive ..///////////
            driveMecanum();

            telemetry.addData("Arm:",arm.getCurrentPosition());
            telemetry.addData("lock:", lock.getPosition());
            telemetry.addData("lift position", lift.getCurrentPosition());
            telemetry.addData("armShort:", armShort.isPressed());
            telemetry.addData("armLong:", armLong.isPressed());
            telemetry.addData("Magnet:", magnet.isPressed());
            telemetry.addData("Rotate position", rotate.getCurrentPosition());
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();
        }
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void armControl(){
        if(armLong.isPressed() || arm.getCurrentPosition() >= 800)
            longArm = true;
        if(armShort.isPressed() )//|| arm.getCurrentPosition() <= 1)     //Temp change arm
            shortArm = true;
        if(gamepad1.right_trigger>0 && !longArm)
        {
            shortArm = false;
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setPower(gamepad1.right_trigger);
            armPosition = arm.getCurrentPosition();
        }
        else if(gamepad1.left_trigger>0 && !shortArm)
        {
            longArm = false;
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setPower(-gamepad1.left_trigger);
            armPosition = arm.getCurrentPosition();
        }
        else
        {
            arm.setTargetPosition(armPosition);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.6);
        }
        if(shortArm || longArm){
            arm.setTargetPosition(armPosition);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.6);
        }

    }
    /*public void wristControl(){
        //////wrist position
//        double wristPosition = (liftPosition.getVoltage()-0.66);
//        if(wristPosition<0)wristPosition = 0;
//        if(wristPosition>0.4)
//            wristPosition=0.4;
//        if(gamepad1.dpad_up || gamepad1.dpad_down)
//             wrist.setPosition(wristPosition);
        //////
        lock.setPosition(0);
    }*/

    public void liftControl(){
        /////lift control../////////////
        if(-gamepad1.left_stick_y>0)
        {
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setPower(-gamepad1.left_stick_y);
            liftHoldPosition = liftPosition.getVoltage();
            liftEncoder = lift.getCurrentPosition();
        }

        else if (-gamepad1.left_stick_y<0 && lift.getCurrentPosition()>0) {
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setPower(-gamepad1.left_stick_y);
            liftHoldPosition = liftPosition.getVoltage();
            liftEncoder = lift.getCurrentPosition();
        }
        else
        {
            liftPID.rest = liftEncoder;
            liftPID.input = lift.getCurrentPosition();
            liftPID.p = 0.005;
            liftPID.d=0.0005;
            liftPID.calculate();
            lift.setPower(liftPID.output);
        }
    }
    public void rotateControl(){
        double power = 0.7;
        if(gamepad2.right_bumper){
            rotatePID.rest= 0;
            rotatePID.input = rotate.getCurrentPosition();
            rotatePID.calculate();
            rotate.setPower(rotatePID.output*3);
            if(Math.abs(rotate.getCurrentPosition())<5)
                lock.setPosition(0.4);
        }
        if(gamepad1.right_stick_x!=0)
        {
            rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lock.setPosition(0.55); //lock open
            rotate.setPower(gamepad1.right_stick_x);
            rotateHoldPosition = rotate.getCurrentPosition();
        }
        else {
            if(rotateHoldPosition!=0){
            rotatePID.rest= rotateHoldPosition;
            rotatePID.input = rotate.getCurrentPosition();
            rotatePID.calculate();
            rotate.setPower(rotatePID.output);
            if(Math.abs(0-rotate.getCurrentPosition())<50)
                lock.setPosition(0.4);//lock tight!
            }
        }
    }
    public void driveMecanum(){
        ///////drive wheels..Mecanum
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad2.left_stick_y*0.8;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad2.left_stick_x*0.8;
        double yaw     =  gamepad2.right_stick_x*0.8;

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
    public void resetArm(){
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       if(!armShort.isPressed() && !armReset)
           arm.setPower(-0.6);
       else
       {
           arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           arm.setTargetPosition(0);
           arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           arm.setPower(0.8);
           armReset = true;
       }
    }

}
