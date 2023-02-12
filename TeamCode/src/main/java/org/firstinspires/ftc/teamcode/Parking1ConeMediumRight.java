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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;



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

@Autonomous(name="Parking 1 Cone Medium Right ", group="Robot")
public class Parking1ConeMediumRight extends LinearOpMode {

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
    private DcMotorEx rightFrontDrive = null;
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
    int liftPositionBeforeDrop;
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
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "exM2");
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
        rotatePID = new PIDController(0.02, 0.0, 0.0);
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
            rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lock.setPosition(0.4);
            claw.setPosition(0.0);
            claw2.setPosition(0.8);

        }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        while (opModeIsActive())
        {
            //claw.setPosition(0.0); //close claws
            //claw2.setPosition(0.8);
            smartDrive(1400, 2370, 50);
            sleep(1000);
            turnToAngle(55, 2);
            sleep(500);
            dropCone();
            while(lift.getCurrentPosition()<2400){
                holdLift(2400);
            }
            turnToAngle(0, 2);
            smartDrive(850, 400, 2);
            sleep(1000);
            turnToAngle(-87, 2);

            if(tagOfInterest.id == 13){
                smartDrive(-1400, 30, 2);
            } else if (tagOfInterest.id == 11){
                smartDrive(1100, 30, 2);
            }
//            while (lift.getCurrentPosition()<1000){
//                holdLift(1000);
//            }
//            dropCone();
            sleep(30000);

            claw.setPosition(0.0); //close claws
            claw2.setPosition(0.8);

            claw.setPosition(0.5);//open claws
            claw2.setPosition(0.0);
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
        int MAX_HEIGHT = 4000;
        if (lift.getCurrentPosition() > MAX_HEIGHT && lift.getTargetPosition() > MAX_HEIGHT) //highest point
            power = 0.6;
        if (Math.abs(lift.getCurrentPosition() - position) > 10 && !lift.isBusy()) {
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setPower(-power * (lift.getCurrentPosition() - position) / Math.abs(lift.getCurrentPosition() - position));
            telemetry.addLine("Lift Busy..");
            telemetry.update();
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

    public void dropCone ()
    {
        int dropFall = 200;
        liftPositionBeforeDrop = lift.getCurrentPosition();
        lift.setTargetPosition(liftPositionBeforeDrop);

        while (lift.getCurrentPosition() > liftPositionBeforeDrop - dropFall)
        {
            holdLift(liftPositionBeforeDrop - dropFall);
            if(lift.getCurrentPosition() > liftPositionBeforeDrop-dropFall*0.5)
            {
                // open claws
                claw.setPosition(0.5);
                claw2.setPosition(0);
            }
        }

        while (lift.getCurrentPosition() < liftPositionBeforeDrop-2)
        {
            holdLift(liftPositionBeforeDrop);
        }
    }
    //
    ///  drive straight by front wheel encoder, starting with an easy acceleration, and ending with
//    an easy deceleration.
    //
    public void smartDrive(int distance, int liftHeight, int armStretch)
    {
        double power = 0.2;
        /////////////////    Reset Encoders     ////////////////////////
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /////////////////////////////////////////////////////////////////
        rightFrontDrive.setTargetPosition(distance);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setTargetPositionTolerance(5);
        // lead with rightFrontDrive, all other wheel motors follow lead.
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive() && rightFrontDrive.isBusy())
        {
            holdLift(liftHeight);
            arm.setTargetPosition(armStretch);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.4);
            if(Math.abs(distance)/2 > Math.abs(rightFrontDrive.getCurrentPosition()))
                power += 0.006;
            else
                power -= 0.006;

            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(rightFrontDrive.getPower());
            leftFrontDrive.setPower(rightFrontDrive.getPower());
            leftBackDrive.setPower(rightFrontDrive.getPower());
        }
        ///when done lock wheels in place.
        lockWheels();
        sleep(100);
    }
}