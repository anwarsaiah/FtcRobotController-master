/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Parking LEFT", group="Robot")
public class ParkingRed2 extends LinearOpMode {

    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable
    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;


    private static final double BACK_CONE = 4000;
    private static final double NORMAL_FLIP = 0.52, FLIPPED = 0.403;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 1;//3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    int wheelsPosition = 0;

    boolean armReset = false;

    AprilTagDetection tagOfInterest = null;


    //////////////////Robot Hardware///////////////////////

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime(), driveTime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx rotate = null;
    private DcMotorEx arm = null;
    private DcMotor lift = null;
    private AnalogInput liftPosition = null;
    private RevTouchSensor magnet = null;
    private PIDController pid = null, rotatePID = null, anglePID = null, rotateRobotPID = null;
    private Servo wrist = null, claw = null, flipHand = null, lock = null, claw2 = null;
    private TouchSensor armShort;
    private TouchSensor armLong;
    double liftHoldPosition = 1.2;//0.989;
    int liftTargetPosition = 1000;
    int rotateHoldPosition = 1;
    YawPitchRollAngles orientation;
    IMU imu;
    boolean liftBusy = false;

    boolean holdingLift = false, firstPass = true;

    //////////////////////////////////////////

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "exM0");
        leftBackDrive = hardwareMap.get(DcMotor.class, "exM1");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "exM2");
        rightBackDrive = hardwareMap.get(DcMotor.class, "exM3");
        rotate = hardwareMap.get(DcMotorEx.class, "rotate");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        lift = hardwareMap.get(DcMotor.class, "lift");
        liftPosition = hardwareMap.get(AnalogInput.class, "liftPosition");
        magnet = hardwareMap.get(RevTouchSensor.class, "magnet");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        lock = hardwareMap.get(Servo.class, "lock");
        flipHand = hardwareMap.get(Servo.class, "flip");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        armShort = hardwareMap.get(TouchSensor.class, "armShort");
        armLong = hardwareMap.get(TouchSensor.class, "armLong");
        pid = new PIDController(0.0002, 0, 0.0001);
        rotatePID = new PIDController(0.02, 0.0, 0.0);
        rotateRobotPID = new PIDController(0.053, 0.0, 0.01);//0.027   or 0.022
        anglePID = new PIDController(0.04, 0, 0.00002);


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        imu = hardwareMap.get(IMU.class, "imu");
        //////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);// w=800 h=448
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.setMsTransmissionInterval(50);
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            anglePID.tolerance = 6;
            imu.resetYaw();
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == 11 || tag.id == 12 || tag.id == 13) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }
////////////////////////////////////////////////\\

            telemetry.addData("lift:", lift.getCurrentPosition());
            //////////////////////////////////////////
            telemetry.update();

            sleep(20);
            runtime.reset();
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flipHand.setPosition(NORMAL_FLIP);//start 0.405 --> end 0.52
            //claw.setPosition(0.47);
            //claw2.setPosition(0.65);//grab cone
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lock.setPosition(0.4);

//            claw.setPosition(0.0);
//            claw2.setPosition(0.8);
            imu.resetYaw();
//            arm.setPositionPIDFCoefficients(0.00001);
//            arm.setTargetPosition(0);
//            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            arm.setPower(0.5);
            //resetArm();

        }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//        Runnable r = new ftcRunnable(this, orientation.getYaw(AngleUnit.DEGREES));
//        new Thread(r).start();
        while (opModeIsActive()) {
//            lift.setTargetPosition(500);
//            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            lift.setPower(1);
            driverStraightTicks(1300,3, 1);

            if(tagOfInterest == null)
            {
                driverStraightTicks(500, 3,1);
                sleep(30000);
            }
            else
            {


                if(tagOfInterest.id == 11)
                {

                    turnToAngle(-90, 3);
                    driverStraightTicks(-1400, 3,1);
                    sleep(30000);

                }
                else if(tagOfInterest.id == 12)
                {
                    driverStraightTicks(500, 1,1);
                    sleep(30000);
                }
                else if(tagOfInterest.id == 13)
                {
                    turnToAngle(90, 3);
                    driverStraightTicks(-1130, 3,1);
                    sleep(30000);




                }
                else // like 3

                {
                    driverStraightTicks(500, 1,1);
                    sleep(30000);
                }
            }
            sleep(30000);
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f meter.", detection.pose.x));
        telemetry.addLine(String.format("Translation Y: %.2f meter.", detection.pose.y));
        telemetry.addLine(String.format("Translation Z: %.2f meter.", detection.pose.z));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void liftControl() {
        /////lift control../////////////
        if (gamepad1.dpad_up) {
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setPower(0.6);
            liftHoldPosition = liftPosition.getVoltage();
        } else if (gamepad1.dpad_down) {
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setPower(-0.6);
            liftHoldPosition = liftPosition.getVoltage();
        } else {
            pid.rest = liftHoldPosition;
            pid.input = liftPosition.getVoltage();
            pid.calculate();
            lift.setPower(pid.output);
        }
    }

    public void rotateControl() {
        if (gamepad1.dpad_left) {
            rotate.setPower(0.9);
            rotateHoldPosition = rotate.getCurrentPosition();
        } else if (gamepad1.dpad_right) {
            {
                rotate.setPower(-0.9);
                rotateHoldPosition = rotate.getCurrentPosition();
            }
        } else {
            if (rotateHoldPosition != 0) {
                rotatePID.rest = rotateHoldPosition;
                rotatePID.input = rotate.getCurrentPosition();
                rotatePID.calculate();
                rotate.setPower(rotatePID.output);
            }
        }
    }

    public void driverStraightTicks(int distance, double timeout, int liftPosition) {
        double power = 0.2;


        orientation = imu.getRobotYawPitchRollAngles();

        /////////////////    Reset Encoders     ////////////////////////
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /////////////////////////////////////////////////////////////////
        rightFrontDrive.setTargetPosition(distance);
        rightBackDrive.setTargetPosition(distance);
        leftFrontDrive.setTargetPosition(distance);
        leftBackDrive.setTargetPosition(distance);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveTime.reset();
        while (opModeIsActive() && driveTime.seconds() < timeout) {
            holdLift(liftPosition);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
            leftFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            telemetry.addData("Driving straigh:", rightFrontDrive.getCurrentPosition());
            telemetry.addData("Time elapsed:", driveTime.seconds());
        }
    }
    public void driverStraightTicks(int distance, double timeout, int liftPosition, int armPosition) {
        double power = 1;

        orientation = imu.getRobotYawPitchRollAngles();

        /////////////////    Reset Encoders     ////////////////////////
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /////////////////////////////////////////////////////////////////
        rightFrontDrive.setTargetPosition(distance);
        rightBackDrive.setTargetPosition(distance);
        leftFrontDrive.setTargetPosition(distance);
        leftBackDrive.setTargetPosition(distance);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveTime.reset();
        while (opModeIsActive() && driveTime.seconds() < timeout) {
            if(driveTime.seconds()<0.3)
                power = 0.01;
            else
                power = 1.0;
            holdLift(liftPosition);
            if(rightFrontDrive.getTargetPosition()>distance/2)
            {
                arm.setTargetPosition(armPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.5);
            }
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
            leftFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            telemetry.addData("Driving straigh:", rightFrontDrive.getCurrentPosition());
            telemetry.addData("Time elapsed:", driveTime.seconds());
        }
    }

    public void slide(int distance, int liftPosition) {
        holdLift(liftPosition);
        double power = 0.8;
        orientation = imu.getRobotYawPitchRollAngles();
        /////////////////    Reset Encoders     ////////////////////////
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /////////////////////////////////////////////////////////////////
        rightFrontDrive.setTargetPosition(-distance);
        rightBackDrive.setTargetPosition(-distance);
        leftFrontDrive.setTargetPosition(distance);
        leftBackDrive.setTargetPosition(distance);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveTime.reset();
        while (opModeIsActive() && Math.abs(rightFrontDrive.getTargetPosition() - rightFrontDrive.getCurrentPosition()) > 2) {
            holdLift(liftPosition);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
            leftFrontDrive.setPower(power);
            leftBackDrive.setPower(power);

        }
    }

    public void driverStraightTicks(int distance, double power, double timeout, int liftPosition) {

        orientation = imu.getRobotYawPitchRollAngles();

        /////////////////    Reset Encoders     ////////////////////////
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /////////////////////////////////////////////////////////////////
        rightFrontDrive.setTargetPosition(distance);
        rightBackDrive.setTargetPosition(distance);
        leftFrontDrive.setTargetPosition(distance);
        leftBackDrive.setTargetPosition(distance);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveTime.reset();
        while (opModeIsActive() && driveTime.seconds() < timeout) {
            holdLift(liftPosition);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
            leftFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            telemetry.addData("Driving straigh:", rightFrontDrive.getCurrentPosition());
            telemetry.addData("Time elapsed:", driveTime.seconds());
        }
    }

    public void driverStraightTicks(int distance, int liftPosition) {
        holdLift(liftPosition);
        double power = 0.8;
        orientation = imu.getRobotYawPitchRollAngles();
        /////////////////    Reset Encoders     ////////////////////////
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /////////////////////////////////////////////////////////////////
        rightFrontDrive.setTargetPosition(distance);
        rightBackDrive.setTargetPosition(distance);
        leftFrontDrive.setTargetPosition(distance);
        leftBackDrive.setTargetPosition(distance);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveTime.reset();
        while (opModeIsActive() && Math.abs(rightFrontDrive.getTargetPosition() - rightFrontDrive.getCurrentPosition())>2) {
            holdLift(liftPosition);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
            leftFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            telemetry.addData("Driving straigh:", rightFrontDrive.getCurrentPosition());
            telemetry.addData("Time elapsed:", driveTime.seconds());
        }
    }


    public void turnToAngle(int angle, double timeout) {
        orientation = imu.getRobotYawPitchRollAngles();
        rotateRobotPID.rest = angle;
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveTime.reset();
        while (opModeIsActive() && driveTime.seconds() < timeout) {
            orientation = imu.getRobotYawPitchRollAngles();
            rotateRobotPID.input = orientation.getYaw(AngleUnit.DEGREES);
            rotateRobotPID.calculate();
            rightFrontDrive.setPower(rotateRobotPID.output);
            rightBackDrive.setPower(rotateRobotPID.output);
            leftFrontDrive.setPower(-rotateRobotPID.output);
            leftBackDrive.setPower(-rotateRobotPID.output);
            telemetry.addData("Angle", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
        leftFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
    }
    public void turnToAngle(int angle, double timeout, int liftPosition) {
        orientation = imu.getRobotYawPitchRollAngles();
        rotateRobotPID.rest = angle;
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveTime.reset();
        while (opModeIsActive() && driveTime.seconds() < timeout) {
            holdLift(liftPosition);
            orientation = imu.getRobotYawPitchRollAngles();
            rotateRobotPID.input = orientation.getYaw(AngleUnit.DEGREES);
            rotateRobotPID.calculate();
            rightFrontDrive.setPower(rotateRobotPID.output);
            rightBackDrive.setPower(rotateRobotPID.output);
            leftFrontDrive.setPower(-rotateRobotPID.output);
            leftBackDrive.setPower(-rotateRobotPID.output);

            telemetry.addData("Angle", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
        leftFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
    }


    public void holdLift(int position) {
        double power = 0.9;
        if (lift.getCurrentPosition() > 4000 && lift.getTargetPosition() > 4000)
            power = 0.3;
        if (Math.abs(lift.getCurrentPosition() - position) > 10 && !lift.isBusy()) {
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            lift.setPower(-power * (lift.getCurrentPosition() - position) / Math.abs(lift.getCurrentPosition() - position));
            telemetry.addLine("Lift Busy..");
        } else {
            lift.setTargetPosition(position);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);
        }
    }

    public void holdLift(double position, double timeout) {
        driveTime.reset();
        holdingLift = false;
        while (opModeIsActive() && driveTime.seconds() < timeout) {
            telemetry.addData("Lift:", lift.getCurrentPosition());
            telemetry.update();
            if (Math.abs(lift.getCurrentPosition() - position) > 0.1 && !holdingLift)
                lift.setPower(-0.8 * (lift.getCurrentPosition() - position) / Math.abs(lift.getCurrentPosition() - position));
            else {
                pid.rest = position;
                pid.input = lift.getCurrentPosition();
                pid.calculate();
                lift.setPower(pid.output);
                holdingLift = true;
            }
        }
    }
    //scoreCone()/////////////////////////////////////////////////////////////////////
    public void scoreCOne(){
        lockWheels();
        lock.setPosition(0.55);   //unlock dish
        /////////     *Simultaneous code*     //////////////////
        //raise lift
        ////rotate dish while lifting
        int CONE_DROP_LIFT_POSITION = 3300;
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        while (runtime.seconds() < 1.5) {
            holdLift(CONE_DROP_LIFT_POSITION);
            //if(lift.getCurrentPosition()>500)
            //flipHand.setPosition(FLIPPED);
            if (lift.getCurrentPosition() > CONE_DROP_LIFT_POSITION-800) {
                orientation = imu.getRobotYawPitchRollAngles();
                rotatePID.rest = 25;
                rotatePID.input = orientation.getYaw(AngleUnit.DEGREES);
                rotatePID.calculate();
                rotate.setPower(rotatePID.output*3);
                telemetry.addData("Current angle", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("lift", lift.getCurrentPosition());
                telemetry.update();
            }
            //stretch arm
            if (lift.getCurrentPosition() > CONE_DROP_LIFT_POSITION - 1500) {
                arm.setTargetPosition(250);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.6);
            }
        }
        rotate.setPower(0);


        ////////  *End of simultaneous code* //////////////////
        driveTime.reset();
        //drop cone
        while (driveTime.seconds() < 0.5) {
            holdLift(CONE_DROP_LIFT_POSITION - 250);
            if (lift.getCurrentPosition()<CONE_DROP_LIFT_POSITION-200) {
                claw.setPosition(0.65);//drop cone
                claw2.setPosition(0.3);
            }

        }

        while (Math.abs(lift.getCurrentPosition() - CONE_DROP_LIFT_POSITION) > 10) {
            holdLift(CONE_DROP_LIFT_POSITION);
        }
        while (lift.getCurrentPosition() > 800) {
            holdLift(800);
            orientation = imu.getRobotYawPitchRollAngles();
            rotatePID.rest = -90;
            rotatePID.input = orientation.getYaw(AngleUnit.DEGREES);
            rotatePID.calculate();
            rotate.setPower(rotatePID.output*3);
            telemetry.addData("Current angle", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("lift", lift.getCurrentPosition());
            telemetry.addData("rotate", rotate.getCurrentPosition());
            telemetry.update();
        }
        rotate.setTargetPosition(0);
        rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotate.setPower(1);
        while(Math.abs(rotate.getCurrentPosition() - 0)>2){
            telemetry.addLine("Centering...");
            holdLift(800);
        }
        lock.setPosition(0.4); //locked
        //turnToAngle(0,1);
        holdLift(800,1);

    }

    public void scoreConeEncoder()
    {
        lockWheels();
        int CONE_DROP_LIFT_POSITION = 2400;

        arm.setTargetPosition(50);
        //lock.setPosition(0.55);   //unlock dish

//        rotate.setTargetPosition(2600);
//        rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rotate.setPower(1);


        while(lift.getCurrentPosition()<CONE_DROP_LIFT_POSITION-2)
        {
            holdLift(CONE_DROP_LIFT_POSITION);
        }

        while (lift.getCurrentPosition()>CONE_DROP_LIFT_POSITION-40){
            holdLift(CONE_DROP_LIFT_POSITION-40);
            if(lift.getCurrentPosition()<CONE_DROP_LIFT_POSITION-30)
            {
                claw.setPosition(0.5);//drop cone
                claw2.setPosition(0.0);
                // sleep(500);
            }

        }
        while (lift.getCurrentPosition() < CONE_DROP_LIFT_POSITION){
            holdLift(CONE_DROP_LIFT_POSITION);
        }

        arm.setTargetPosition(0);
        while (lift.getCurrentPosition()<2700){
            holdLift(2700);
        }
//        lock.setPosition(0.4); //locked
    }

    private void lockWheels() {

        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition());
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setPower(1);

        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition());
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setPower(1);

        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition());
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setPower(1);

        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition());
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setPower(1);
    }
    //scoreCone() //////////////////////////////////////////////////////////////////

    public void getStackedCone(int liftHeight){

        driverStraightTicks(800,0.8,  liftHeight); //0.65
        runtime.reset();
        while (runtime.seconds()<0.65)
        {
            telemetry.addData("Angle", orientation.getYaw(AngleUnit.DEGREES));
            holdLift(liftHeight);
            arm.setTargetPosition(195);
            if(runtime.seconds()>0.4)
            {
                claw.setPosition(0.0);
                claw2.setPosition(0.5);
            }
        }
        arm.setTargetPosition(88);
        sleep(100);
        while (lift.getCurrentPosition()<liftHeight+200)
            holdLift(liftHeight+200);
        while (lift.getCurrentPosition()<liftHeight+400)
            holdLift(liftHeight+400);
        //driverStraightTicks(-900, 2, liftHeight+50);

        driverStraightTicks(-800, 0.5, 0.8, liftHeight+400);//0.65
    }



    ///////////////////////////////////////////////////////////////////////////////////////
    /// Gyro Based Auto Drive//////////////


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

    public  class ftcRunnable implements  Runnable{
        LinearOpMode opMode;
        double value;
        public ftcRunnable(LinearOpMode opMode, double value){
            this.opMode = opMode;
            this.value = value;
        }
        public void run(){
            opMode.telemetry.addData("Angle", value);
            opMode.telemetry.update();
        }
    }

}