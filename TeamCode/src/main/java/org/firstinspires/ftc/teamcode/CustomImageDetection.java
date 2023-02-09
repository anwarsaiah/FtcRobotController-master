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

@Autonomous(name="Autonomous With Detection ", group="Robot")
public class CustomImageDetection extends LinearOpMode {

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


    AprilTagDetection tagOfInterest = null;


    //////////////////Robot Hardware///////////////////////

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime(), driveTime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx rotate = null;
    private DcMotor arm = null;
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
        arm = hardwareMap.get(DcMotor.class, "arm");
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
        rotatePID = new PIDController(0.035, 0.0, 0.002);//  0.04, 0.0, 0.0001
        rotateRobotPID = new PIDController(0.027, 0.0, 0.0);
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
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
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
            lock.setPosition(0.4);


        }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        while (opModeIsActive()) {

            runtime.reset();
            lockWheels();
            //grab cone
            while (runtime.seconds()<2)
            {
                holdLift(50);
                if(lift.getCurrentPosition()>10)
                {
                    claw.setPosition(0.0);
                    claw2.setPosition(1);
                    break;
                }
            }

            lift.setTargetPosition(500);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);


            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);
            //driverStraightTicks(2250, 2, 500);
            driveStraight(0.95, 43.5, 0, 300);
            turnToAngle(0, 0.55);
            turnToAngle(-90, 0.8);
            //lift.setTargetPosition(400);
            driverStraightTicks(-300, 0.25, 500);

            //unlock dish

            imu.resetYaw();
            orientation = imu.getRobotYawPitchRollAngles();

            scoreCOne();
            getStackedCone(900);
            scoreCOne();
            getStackedCone(850);
            scoreCOne();
            getStackedCone(800);
            scoreCOne();
            getStackedCone(750);
            scoreCOne();
            getStackedCone(700);
            scoreCOne();
            getStackedCone(650);

            sleep(25000);


/////////////////////////////////////////Parking////////////////////////////////////////////////////

            if (tagOfInterest != null) {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();//GitHub token
            }//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
            if (tagOfInterest != null) {
                if (tagOfInterest.id == 11 || tagOfInterest == null) {
                    //park left.
                    //telemetry.addLine("We should park left.");
                    //telemetry.update();
                }
                if (tagOfInterest.id == 12) {
                    //park middle.
                    // telemetry.addLine("We should park in the middle.");
                    //telemetry.update();
                }
                if (tagOfInterest.id == 13) {
                    //park right.
                    // telemetry.addLine("We should park right.");
                    //telemetry.update();
                }
            }
            orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("distance", rightFrontDrive.getCurrentPosition());
            telemetry.addData("Lift Position:", liftPosition.getVoltage());
            telemetry.addData("Arm Short", armShort.isPressed());
            telemetry.addData("Arm Long", armLong.isPressed());
//            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
//            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
//            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
//            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
//            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();
            /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
            //while (opModeIsActive()) {sleep(20);}
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
        double power = 0.8;

        imu.resetYaw();
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
        double p = 0.0;
        driveTime.reset();
        while (opModeIsActive() && driveTime.seconds() < timeout) {
            holdLift(liftPosition);
            rightFrontDrive.setPower(power-orientation.getYaw(AngleUnit.DEGREES)*p);
            rightBackDrive.setPower(power-orientation.getYaw(AngleUnit.DEGREES)*p);
            leftFrontDrive.setPower(power+orientation.getYaw(AngleUnit.DEGREES)*p);
            leftBackDrive.setPower(power+orientation.getYaw(AngleUnit.DEGREES)*p);
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
        while (runtime.seconds() < 2.5) {
            holdLift(CONE_DROP_LIFT_POSITION);
            //if(lift.getCurrentPosition()>500)
            //flipHand.setPosition(FLIPPED);
            if (lift.getCurrentPosition() > CONE_DROP_LIFT_POSITION-800) {
                orientation = imu.getRobotYawPitchRollAngles();
                rotatePID.rest = 100;
                rotatePID.input = orientation.getYaw(AngleUnit.DEGREES);
                rotatePID.calculate();
                rotate.setPower(rotatePID.output);
                telemetry.addData("Current angle", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("lift", lift.getCurrentPosition());
                telemetry.update();
            }
            //stretch arm
            if (lift.getCurrentPosition() > CONE_DROP_LIFT_POSITION - 1500) {
                arm.setTargetPosition(400);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.6);
            }
        }
        rotate.setPower(0);


        ////////  *End of simultaneous code* //////////////////
        driveTime.reset();
        //drop cone
        while (driveTime.seconds() < 2) {
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
            rotatePID.rest = 0;
            rotatePID.input = orientation.getYaw(AngleUnit.DEGREES);
            rotatePID.calculate();
            rotate.setPower(rotatePID.output);
            telemetry.addData("Current angle", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("lift", lift.getCurrentPosition());
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
        turnToAngle(0,1);
        holdLift(800,1);

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
        driveStraight(0.6, 16, 0.0, liftHeight);
        runtime.reset();
        while (runtime.seconds()<1.5)
        {
            holdLift(liftHeight);
            arm.setTargetPosition(1000);
            if(lift.getCurrentPosition()>liftHeight-5)
            {
                claw.setPosition(0.0);
                claw2.setPosition(1);
            }
        }
        //driverStraightTicks(-900, 2, liftHeight+50);
        driveStraight(0.6, -16, 0.0, liftHeight);
    }
    ///////////////////////////////////////////////////////////////////////////////////////
    /// Gyro Based Auto Drive//////////////
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading,
                              int liftHeight) {  //floor squares 22.75" each

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            holdLift(liftHeight);
            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
            rightTarget = rightFrontDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftFrontDrive.setTargetPosition(leftTarget);
            rightFrontDrive.setTargetPosition(rightTarget);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }


    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftFrontDrive.setPower(leftSpeed);
        leftBackDrive.setPower(leftSpeed);
        rightFrontDrive.setPower(rightSpeed);
        rightBackDrive.setPower(rightSpeed);
    }


    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }
    public double getRawHeading() {
        orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}