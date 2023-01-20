/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.R;

/**
 * This particular OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: Teleop POV", group="Robot")
//@Disabled
public class RobotTeleopPOV_Linear extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor  frontLeft = null;
    public DcMotor  frontRight = null;
    public DcMotor  backLeft = null;
    public DcMotor  backRight = null;
    public DcMotor  elevator1     = null;
    public DcMotorEx  elevator2    = null;
    public Servo    intakeServo   = null;
    public Servo    elevatorServo   = null;
    public Servo    cone   = null;
    public Servo    elevatorServo2   = null;
    public DcMotorEx dish = null;
    public DcMotorEx intake = null;
    public TouchSensor start = null;
    public TouchSensor end = null;
    double clawOffset = 0;

    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    boolean gamePad_a = false;
    public boolean liftingElevator;
    ElapsedTime elapsedTime = new ElapsedTime();
    ElapsedTime scoreTime = new ElapsedTime();
    boolean endReached, startReached, firstPass, intakeUP;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;

        // Define and Initialize Motors

        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");
        elevator1 = hardwareMap.get(DcMotor.class, "elevator1");
        elevator2 = hardwareMap.get(DcMotorEx.class, "elevator2");
        intakeServo = hardwareMap.get(Servo.class, "intakeservo");
        elevatorServo = hardwareMap.get(Servo.class, "elevatorservo");
        cone = hardwareMap.get(Servo.class, "cone");
        elevatorServo2 = hardwareMap.get(Servo.class, "elevatorservo2");
        dish = hardwareMap.get(DcMotorEx.class, "dish");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        start = hardwareMap.get(TouchSensor.class, "start");
        end = hardwareMap.get(TouchSensor.class, "end");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);



        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.addData("cone position", cone.getPosition());

        telemetry.update();
        //////
        elevator2.setDirection(DcMotor.Direction.REVERSE);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dish.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        endReached = false;
        startReached = false;
        firstPass = true;
        liftingElevator = false;
        intakeUP=false;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        elapsedTime.reset();
        scoreTime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

//            if(intake.getCurrentPosition()>100)
//                intakeUP = true;


            if(gamepad1.a)
            {
                gamePad_a = ! gamePad_a;
                liftingElevator = true;
                sleep(500);
                scoreTime.reset();
//                if(gamePad_a)
//                {
//                    cone.setPosition(0.7);
//                    intake.setTargetPosition(140);
//                    intake.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(3.5, 0.0, 0, 0));
//                    intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    intake.setPower(1);
//                }
            }
            if(gamePad_a)
            {

                scoreCone();
//                if(elapsedTime.seconds()>6 && elapsedTime.seconds()<10)
//                {
//                    liftElevator();
//                }
//                if(intakeUP)
//                {
//                    dish.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//                    if(!endReached)
//                    {
//                        dish.setPower(-0.4);
//                        elevatorServo.setPosition(200);
//                        telemetry.addData("Dish spinning..", 999);
//                    }
//                    else
//                    {
//                        //first time only..
//                        if(firstPass)
//                        {
//                            dish.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                            dish.setPower(0);
//                            elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                            firstPass =false;
//                            liftingElevator = true;
//                        }
//                        dishHold();
//                        //elevatorServo.setPosition(200);
//
//                    }
//
//                }

            }

            elevator1.setPower(-elevator2.getPower());
            drive =  gamepad1.left_stick_y;
            turn  =  -gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives.
            frontLeft.setPower(left);
            backLeft.setPower(left);
            frontRight.setPower(right);
            backRight.setPower(right);

            // Send telemetry message to signify robot running;
            //telemetry.addData("elevator1",  "Offset = "+ elevator1.getCurrentPosition());
            telemetry.addData("elevator2",  "Offset = "+ elevator2.getCurrentPosition());
            // telemetry.addData("start Pressed", start.isPressed());
            //telemetry.addData("end Pressed", end.isPressed());
            //telemetry.addData("dish", "Offset = "+dish.getCurrentPosition());
            //telemetry.addData("intake", "Offset = "+intake.getCurrentPosition());
//            telemetry.addData("left",  "%.2f", left);
//            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }

    private void liftElevator(){
        elevator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(elevator2.getCurrentPosition()>-1250 && liftingElevator){
            elevator2.setPower(-1);
            elevator1.setPower(-1);
        }
        else {
            elevator2.setTargetPosition(-1260);
            elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator2.setPower(-0.5);
            liftingElevator = false;
        }
    }
    private void dishHold(){
        dish.setTargetPosition(0);
        dish.setPositionPIDFCoefficients(3);
        dish.setTargetPositionTolerance(5);
        dish.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dish.setPower(-0.6);
    }

    ///////score function ////////////////////////////
    void scoreCone(){

        if(scoreTime.seconds()<2)
        {
            cone.setPosition(0.9);
            intake.setTargetPosition(140);
            intake.setPositionPIDFCoefficients(3);
            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.setPower(1);
            telemetry.addData("intake", 1);
            telemetry.update();
        }
        if(scoreTime.seconds()>2 && scoreTime.seconds()<5)
        {
            telemetry.addData("dish", 2);
            telemetry.update();
            elevatorServo.setPosition(1);
            if(end.isPressed())
                endReached = true;
            if(endReached)
                dishHold();
            else {
                dish.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                dish.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                dish.setPower(-0.4);
            }
        }
        if(scoreTime.seconds()>5 && scoreTime.seconds()<8)

        {
            telemetry.addData("elevator", 3);
            telemetry.update();
            liftElevator();
        }
        if(scoreTime.seconds()>9 && scoreTime.seconds()<10)
        {
            telemetry.addData("cone", 4);
            telemetry.update();

            cone.setPosition(1);
        }
        if(scoreTime.seconds()>10 && scoreTime.seconds()<11)
        {
            telemetry.addData("elevator", 5);
            telemetry.update();
            elevator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elevator2.setPower(0.5);
            elevator1.setPower(0.5);
        }
        if(scoreTime.seconds()>11 && scoreTime.seconds()<14)
        {
            telemetry.addData("dish", 6);
            telemetry.update();
            elevator2.setPower(0.0);
            elevator1.setPower(0.0);
            liftingElevator =false;
            dish.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dish.setPower(0.4);
            if(start.isPressed())
                startReached = true;
            if(startReached)
            {
                dishHold();
                intake.setTargetPosition(0);
                intake.setPower(-0.5);
            }
        }
        if(scoreTime.seconds()>14 && scoreTime.seconds()<30){
            telemetry.addData("intake", 7);
            telemetry.update();
            intake.setTargetPosition(0);
            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.setPower(1);
        }
        gamePad_a = false;
    }


}
