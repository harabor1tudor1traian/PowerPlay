
package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constant;
import org.firstinspires.ftc.teamcode.demoBot.Hardware_Demo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.examples.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.ElementDetection;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Auto_Left", group="Roadrunner")
public class Auto_Left extends LinearOpMode {

    private enum robotStage{
        scan,
        preLoad,
        stack1,
        cone1,
        stack2,
        cone2,
        stack3,
        cone3,
        stack4,
        cone4,
        stack5,
        cone5,
        park,
        stopRobot}
    private final static robotStage[] robotStageValues = robotStage.values();
    private int programstage=0;

    private int  nextStage(int ordinal){
        return ordinal;
    }

    private int  nextStage(){
        return nextStage(programstage+1);
    }

    private int  thisStage(){
        return nextStage(programstage);
    }

    Hardware robot = new Hardware();
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    private ElapsedTime autoTimer = new ElapsedTime();
    private ElapsedTime cycleRuntime = new ElapsedTime();
    private ElapsedTime detectionTime = new ElapsedTime();
    private Constant ct = new Constant();
    private int parkZone = 2;
    boolean movingArm = false;
    boolean cycleTimer = true;
    boolean cycle4 = false;
    boolean cycle5 = false;
    boolean tagFound = false;
    double cycleTime;

    double leftPos = Constant.leftArmCollect;
    double rightPos = Constant.rightArmCollect;
    double clawPos = Constant.collect;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

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

    int id1 = 1;
    int id2 = 2;
    int id3 = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        Pose2d startPose = new Pose2d(-90, -159, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        resetEncoders();
        Trajectory preLoad;
        TrajectorySequence stack1;
        Trajectory cone1;
        Trajectory stack2;
        Trajectory cone2;
        Trajectory stack3;
        Trajectory cone3;
        Trajectory stack4;
        Trajectory cone4;
        Trajectory stack5;
        Trajectory cone5;
        Trajectory park = null;

        preLoad = drive.trajectoryBuilder(new Pose2d(), false)
                .splineTo(new Vector2d(-81,81), Math.toRadians(45))
                .build();
        stack1 = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(60)
                .splineTo(new Vector2d(-120,-30), Math.toRadians(180))
                .build();
        cone1 = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(-81,-39), Math.toRadians(135))
                .build();
        stack2 = drive.trajectoryBuilder(new Pose2d(), false)
                .splineTo(new Vector2d(-120,-30), Math.toRadians(180))
                .build();
        cone2 = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(-81,-39), Math.toRadians(135))
                .build();
        stack3 = drive.trajectoryBuilder(new Pose2d(), false)
                .splineTo(new Vector2d(-120,-30), Math.toRadians(180))
                .build();
        cone3 = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(-81,-39), Math.toRadians(135))
                .build();
        stack4 = drive.trajectoryBuilder(new Pose2d(), false)
                .splineTo(new Vector2d(-120,-30), Math.toRadians(180))
                .build();
        cone4 = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(-81,-39), Math.toRadians(135))
                .build();
        stack5 = drive.trajectoryBuilder(new Pose2d(), false)
                .splineTo(new Vector2d(-120,-30), Math.toRadians(180))
                .build();
        cone5 = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(-81,-39), Math.toRadians(135))
                .build();
        ElementDetection detectElement = new ElementDetection(hardwareMap, telemetry);

        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                robot.motorFl.getCurrentPosition(),
                robot.motorBl.getCurrentPosition(),
                robot.motorBr.getCurrentPosition(),
                robot.motorFr.getCurrentPosition());
        telemetry.update();

        waitForStart();
        detectionTime.reset();

        while (opModeIsActive()){
            telemetry.addData("ParkZone:", parkZone);
            telemetry.addData("Stage:", programstage);
            telemetry.update();
            drive.update();
            switch (robotStageValues[programstage]){
                case scan:
                    while (!tagFound && detectionTime.seconds() > 0.2) {
                        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                        if (currentDetections.size() != 0) {

                            for (AprilTagDetection tag : currentDetections) {
                                if (tag.id == id1) {
                                    parkZone = 1;
                                    tagFound = true;
                                    break;
                                }
                                else if (tag.id == id3){
                                    parkZone = 3;
                                    tagFound = true;
                                    break;
                                }
                                else {
                                    parkZone = 2;
                                    tagFound = true;
                                    break;
                                }
                            }
                        }
                    }
                    if (parkZone == 1){
                        park = drive.trajectoryBuilder(new Pose2d())
                                .splineTo(new Vector2d(-150,-30), Math.toRadians(180))
                                .build();
                    }
                    else if (parkZone == 2){
                        park = drive.trajectoryBuilder(new Pose2d())
                                .splineTo(new Vector2d(-90,-30), Math.toRadians(90))
                                .build();
                    }
                    else
                        park = drive.trajectoryBuilder(new Pose2d())
                                .splineTo(new Vector2d(-30, -30), Math.toRadians(90))
                                .build();
                    programstage = nextStage();
                    break;

                case preLoad:
                    moveArm(Constant.leftArmDrop);
                    drive.followTrajectoryAsync(preLoad);
                    if (drive.isBusy())
                        programstage = thisStage();
                    else {
                        Hardware.mainClaw.setPosition(Constant.openClaw);
                        sleep(500);
                        programstage = nextStage();
                    }
                    break;

                case stack1:
                    moveArm(Constant.leftArmCone1);
                    drive.followTrajectorySequenceAsync(stack1);
                    if (drive.isBusy() || movingArm)
                        programstage = thisStage();
                    else {
                        Hardware.mainClaw.setPosition(Constant.closedClaw);
                        sleep(500);
                        programstage = nextStage();
                    }
                    break;

                case cone1:
                    moveArm(Constant.leftArmDropBack);
                    drive.followTrajectoryAsync(cone1);
                    if (drive.isBusy() || movingArm)
                        programstage = thisStage();
                    else {
                        Hardware.mainClaw.setPosition(Constant.openClaw);
                        sleep(500);
                        programstage = nextStage();
                    }
                    break;
                case stack2:
                    if (cycleTimer)
                        cycleRuntime.reset();
                    moveArm(Constant.leftArmCone2);
                    drive.followTrajectoryAsync(stack2);
                    if (drive.isBusy() || movingArm)
                        programstage = thisStage();
                    else {
                        Hardware.mainClaw.setPosition(Constant.closedClaw);
                        sleep(500);
                        programstage = nextStage();
                    }
                    break;
                case cone2:
                    moveArm(Constant.leftArmDropBack);
                    drive.followTrajectoryAsync(cone2);
                    if (drive.isBusy() || movingArm)
                        programstage = thisStage();
                    else {
                        Hardware.mainClaw.setPosition(Constant.openClaw);
                        sleep(500);
                        programstage = nextStage();
                    }
                    break;
                case stack3:
                    cycleTime = cycleRuntime.seconds();
                    moveArm(Constant.leftArmCone3);
                    drive.followTrajectoryAsync(stack3);
                    if (drive.isBusy() || movingArm)
                        programstage = thisStage();
                    else {
                        Hardware.mainClaw.setPosition(Constant.closedClaw);
                        sleep(500);
                        programstage = nextStage();
                    }
                    break;
                case cone3:
                    moveArm(Constant.leftArmDropBack);
                    drive.followTrajectoryAsync(cone3);
                    if (drive.isBusy() || movingArm)
                        programstage = thisStage();
                    else {
                        Hardware.mainClaw.setPosition(Constant.openClaw);
                        sleep(500);
                        programstage = nextStage();
                    }
                    break;
                case stack4:
                    if (cycleTime + 3 < 30 - autoTimer.seconds() && !cycle4)
                        cycle4 = true;
                    if (cycle4){
                        moveArm(Constant.leftArmCone4);
                        drive.followTrajectoryAsync(stack4);
                        if (drive.isBusy() || movingArm)
                            programstage = thisStage();
                        else {
                            Hardware.mainClaw.setPosition(Constant.closedClaw);
                            sleep(500);
                            programstage = nextStage();
                        }
                        break;
                    }
                    else {
                        programstage = nextStage();
                        break;
                    }
                case cone4:
                    if (cycle4) {
                        moveArm(Constant.leftArmDropBack);
                        drive.followTrajectoryAsync(cone4);
                        if (drive.isBusy() || movingArm)
                            programstage = thisStage();
                        else {
                            Hardware.mainClaw.setPosition(Constant.openClaw);
                            sleep(500);
                            programstage = nextStage();
                        }
                        break;
                    }
                    else{
                        programstage = nextStage();
                        break;
                    }
                case stack5:
                    if (cycleTime + 3 < 30 - autoTimer.seconds() && !cycle5)
                        cycle5 = true;
                    if (cycle5) {
                        moveArm(Constant.leftArmCone5);
                        drive.followTrajectoryAsync(stack5);
                        if (drive.isBusy() || movingArm)
                            programstage = thisStage();
                        else {
                            Hardware.mainClaw.setPosition(Constant.closedClaw);
                            sleep(500);
                            programstage = nextStage();
                        }
                        break;
                    }
                    else {
                        programstage = nextStage();
                        break;
                    }
                case cone5:
                    if (cycle5) {
                        moveArm(Constant.leftArmDropBack);
                        drive.followTrajectoryAsync(cone5);
                        if (drive.isBusy() || movingArm)
                            programstage = thisStage();
                        else {
                            Hardware.mainClaw.setPosition(Constant.openClaw);
                            sleep(500);
                            programstage = nextStage();
                        }
                        break;
                    }
                    else {
                        programstage = nextStage();
                        break;
                    }
                case park:
                    moveArm(Constant.collect);
                    drive.followTrajectoryAsync(park);
                    if (drive.isBusy() || movingArm)
                        programstage = thisStage();
                    else programstage = nextStage();
                    break;
                case stopRobot:
                    terminateOpModeNow();
                    break;
            }
        }
    }

    public void driveForward(double distance, int leftDif, int rightDif) {
        if (opModeIsActive()) {
            int flTarget, blTarget, brTarget, frTarget;
            flTarget = robot.motorFl.getCurrentPosition() + (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));
            blTarget = robot.motorBl.getCurrentPosition() + (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));
            brTarget = robot.motorBr.getCurrentPosition() + (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));
            frTarget = robot.motorFr.getCurrentPosition() + (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));

            robot.motorFl.setTargetPosition(flTarget);
            robot.motorBl.setTargetPosition(blTarget);
            robot.motorBr.setTargetPosition(brTarget);
            robot.motorFr.setTargetPosition(frTarget);

            robot.motorFl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFr.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            for (int i = 10; i <= Constant.maxVelocityDrivetrain; i+=10) {
                robot.motorFl.setVelocity(i+leftDif, AngleUnit.DEGREES);
                robot.motorBl.setVelocity(i+leftDif, AngleUnit.DEGREES);
                robot.motorBr.setVelocity(i+rightDif, AngleUnit.DEGREES);
                robot.motorFr.setVelocity(i+rightDif, AngleUnit.DEGREES);
                sleep(50);
            }

            while (robot.motorFl.isBusy() || robot.motorBl.isBusy() || robot.motorBr.isBusy() || robot.motorFr.isBusy()) {
                telemetry.addData("Current position: ", "%2d : %2d : %2d : %2d",
                        robot.motorFl.getCurrentPosition(),
                        robot.motorBl.getCurrentPosition(),
                        robot.motorBr.getCurrentPosition(),
                        robot.motorFr.getCurrentPosition());
                telemetry.update();
                if (((Hardware_Demo.motorFl.getCurrentPosition()/ Constant.ticksPerRev* Constant.wheelDiameter * 3.1415)-Hardware_Demo.motorFl.getTargetPosition()/ Constant.ticksPerRev* Constant.wheelDiameter*3.1415)>5)
                    break;
            }

            robot.motorFl.setVelocity(0, AngleUnit.DEGREES);
            robot.motorBl.setVelocity(0, AngleUnit.DEGREES);
            robot.motorBr.setVelocity(0, AngleUnit.DEGREES);
            robot.motorFr.setVelocity(0, AngleUnit.DEGREES);

            resetEncoders();

        }

    }

    public void driveBackward(double distance, int leftDif, int rightDif) {
        if (opModeIsActive()) {
            int flTarget, blTarget, brTarget, frTarget;
            flTarget = robot.motorFl.getCurrentPosition() - (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));
            blTarget = robot.motorBl.getCurrentPosition() - (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));
            brTarget = robot.motorBr.getCurrentPosition() - (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));
            frTarget = robot.motorFr.getCurrentPosition() - (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));

            robot.motorFl.setTargetPosition(flTarget);
            robot.motorBl.setTargetPosition(blTarget);
            robot.motorBr.setTargetPosition(brTarget);
            robot.motorFr.setTargetPosition(frTarget);

            robot.motorFl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            for (int i = 10; i <= Constant.maxVelocityDrivetrain; i+=10) {
                robot.motorFl.setVelocity(-i-leftDif, AngleUnit.DEGREES);
                robot.motorBl.setVelocity(-i-leftDif, AngleUnit.DEGREES);
                robot.motorBr.setVelocity(-i-rightDif, AngleUnit.DEGREES);
                robot.motorFr.setVelocity(-i-rightDif, AngleUnit.DEGREES);
                sleep(50);
            }

            while (robot.motorFl.isBusy() || robot.motorBl.isBusy() || robot.motorBr.isBusy() || robot.motorFr.isBusy()) {
                telemetry.addData("Current position: ", "%2d : %2d : %2d : %2d",
                        robot.motorFl.getCurrentPosition(),
                        robot.motorBl.getCurrentPosition(),
                        robot.motorBr.getCurrentPosition(),
                        robot.motorFr.getCurrentPosition());
                telemetry.update();
                if (((Hardware_Demo.motorFl.getCurrentPosition()/ Constant.ticksPerRev* Constant.wheelDiameter * 3.1415)-Hardware_Demo.motorFl.getTargetPosition()/ Constant.ticksPerRev* Constant.wheelDiameter*3.1415)>5)
                    break;
            }
            robot.motorFl.setVelocity(0, AngleUnit.DEGREES);
            robot.motorBl.setVelocity(0, AngleUnit.DEGREES);
            robot.motorBr.setVelocity(0, AngleUnit.DEGREES);
            robot.motorFr.setVelocity(0, AngleUnit.DEGREES);

            resetEncoders();

        }
    }

    public void slideRight(double distance) {
        if (opModeIsActive()) {
            int flTarget, blTarget, brTarget, frTarget;
            flTarget = robot.motorFl.getCurrentPosition() + (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));
            blTarget = robot.motorBl.getCurrentPosition() - (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));
            brTarget = robot.motorBr.getCurrentPosition() + (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));
            frTarget = robot.motorFr.getCurrentPosition() - (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));

            robot.motorFl.setTargetPosition(flTarget);
            robot.motorBl.setTargetPosition(blTarget);
            robot.motorBr.setTargetPosition(brTarget);
            robot.motorFr.setTargetPosition(frTarget);

            robot.motorFl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            for (int i = 10; i <= Constant.maxVelocityDrivetrain; i+=10) {
                robot.motorFl.setVelocity(i, AngleUnit.DEGREES);
                robot.motorBl.setVelocity(-i, AngleUnit.DEGREES);
                robot.motorBr.setVelocity(i, AngleUnit.DEGREES);
                robot.motorFr.setVelocity(-i, AngleUnit.DEGREES);
                sleep(50);

            }

            while (robot.motorFl.isBusy()  && robot.motorBl.isBusy()  && robot.motorBr.isBusy()  && robot.motorFr.isBusy()) {
                telemetry.addData("Current position: ", "%2d : %2d : %2d : %2d",
                        robot.motorFl.getCurrentPosition(),
                        robot.motorBl.getCurrentPosition(),
                        robot.motorBr.getCurrentPosition(),
                        robot.motorFr.getCurrentPosition());
                telemetry.update();
                if (((Hardware_Demo.motorFl.getCurrentPosition()/ Constant.ticksPerRev* Constant.wheelDiameter * 3.1415)-Hardware_Demo.motorFl.getTargetPosition()/ Constant.ticksPerRev* Constant.wheelDiameter*3.1415)>5)
                    break;
            }

            robot.motorFl.setVelocity(0, AngleUnit.DEGREES);
            robot.motorBl.setVelocity(0, AngleUnit.DEGREES);
            robot.motorBr.setVelocity(0, AngleUnit.DEGREES);
            robot.motorFr.setVelocity(0, AngleUnit.DEGREES);

            resetEncoders();

        }

    }


    public void slideLeft(double distance) {
        if (opModeIsActive()) {
            int flTarget, blTarget, brTarget, frTarget;
            flTarget = robot.motorFl.getCurrentPosition() - (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));
            blTarget = robot.motorBl.getCurrentPosition() + (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));
            brTarget = robot.motorBr.getCurrentPosition() - (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));
            frTarget = robot.motorFr.getCurrentPosition() + (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));

            robot.motorFl.setTargetPosition(flTarget);
            robot.motorBl.setTargetPosition(blTarget);
            robot.motorBr.setTargetPosition(brTarget);
            robot.motorFr.setTargetPosition(frTarget);

            robot.motorFl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            for (int i = 10; i <= Constant.maxVelocityDrivetrain; i+=10) {
                robot.motorFl.setVelocity(-i, AngleUnit.DEGREES);
                robot.motorBl.setVelocity(i, AngleUnit.DEGREES);
                robot.motorBr.setVelocity(-i, AngleUnit.DEGREES);
                robot.motorFr.setVelocity(i, AngleUnit.DEGREES);
                sleep(50);
            }

            while (robot.motorFl.isBusy() && robot.motorBl.isBusy() && robot.motorBr.isBusy() && robot.motorFr.isBusy()) {
                telemetry.addData("Current position: ", "%2d : %2d : %2d : %2d",
                        robot.motorFl.getCurrentPosition(),
                        robot.motorBl.getCurrentPosition(),
                        robot.motorBr.getCurrentPosition(),
                        robot.motorFr.getCurrentPosition());
                telemetry.update();
                if (((Hardware_Demo.motorFl.getCurrentPosition()/ Constant.ticksPerRev* Constant.wheelDiameter * 3.1415)-Hardware_Demo.motorFl.getTargetPosition()/ Constant.ticksPerRev* Constant.wheelDiameter*3.1415)>5)
                    break;
            }

            robot.motorFl.setVelocity(0, AngleUnit.DEGREES);
            robot.motorBl.setVelocity(0, AngleUnit.DEGREES);
            robot.motorBr.setVelocity(0, AngleUnit.DEGREES);
            robot.motorFr.setVelocity(0, AngleUnit.DEGREES);

            resetEncoders();
        }
    }

    public void rotateRight(int degrees){
        if (opModeIsActive()){
            int flTarget, blTarget, brTarget, frTarget;
            int distance = (int)((degrees * 2 * Constant.chassis_width * 3.1415)/360);
            flTarget = robot.motorFl.getCurrentPosition() + (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));
            blTarget = robot.motorBl.getCurrentPosition() + (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));
            brTarget = robot.motorBr.getCurrentPosition() - (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));
            frTarget = robot.motorFr.getCurrentPosition() - (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));

            robot.motorFl.setTargetPosition(flTarget);
            robot.motorBl.setTargetPosition(blTarget);
            robot.motorBr.setTargetPosition(brTarget);
            robot.motorFr.setTargetPosition(frTarget);

            robot.motorFl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.motorFl.setVelocity(Constant.maxVelocityDrivetrain);
            robot.motorBl.setVelocity(Constant.maxVelocityDrivetrain);
            robot.motorBr.setVelocity(-Constant.maxVelocityDrivetrain);
            robot.motorFr.setVelocity(-Constant.maxVelocityDrivetrain);

            while (robot.motorFl.isBusy()  || robot.motorBl.isBusy()  || robot.motorBr.isBusy()  || robot.motorFr.isBusy()) {
                telemetry.addData("Current position: ", "%2d : %2d : %2d : %2d",
                        robot.motorFl.getCurrentPosition(),
                        robot.motorBl.getCurrentPosition(),
                        robot.motorBr.getCurrentPosition(),
                        robot.motorFr.getCurrentPosition());
                telemetry.update();
            }
            robot.motorFl.setVelocity(0);
            robot.motorBl.setVelocity(0);
            robot.motorBr.setVelocity(0);
            robot.motorFr.setVelocity(0);

            resetEncoders();

        }
    }

    public void rotateLeft(int degrees){
        if (opModeIsActive()){
            int flTarget, blTarget, brTarget, frTarget;
            int distance = (int)((degrees * 2 * Constant.chassis_width * 3.1415)/360);
            flTarget = robot.motorFl.getCurrentPosition() - (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));
            blTarget = robot.motorBl.getCurrentPosition() - (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));
            brTarget = robot.motorBr.getCurrentPosition() + (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));
            frTarget = robot.motorFr.getCurrentPosition() + (int) ((Constant.ticksPerRev * distance) / (Constant.wheelDiameter * 3.1415));

            robot.motorFl.setTargetPosition(flTarget);
            robot.motorBl.setTargetPosition(blTarget);
            robot.motorBr.setTargetPosition(brTarget);
            robot.motorFr.setTargetPosition(frTarget);

            robot.motorFl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.motorFl.setVelocity(-Constant.maxVelocityDrivetrain);
            robot.motorBl.setVelocity(-Constant.maxVelocityDrivetrain);
            robot.motorBr.setVelocity(Constant.maxVelocityDrivetrain);
            robot.motorFr.setVelocity(Constant.maxVelocityDrivetrain);

            while (robot.motorFl.isBusy()  || robot.motorBl.isBusy()  || robot.motorBr.isBusy()  || robot.motorFr.isBusy()) {
                telemetry.addData("Current position: ", "%2d : %2d : %2d : %2d",
                        robot.motorFl.getCurrentPosition(),
                        robot.motorBl.getCurrentPosition(),
                        robot.motorBr.getCurrentPosition(),
                        robot.motorFr.getCurrentPosition());
                telemetry.update();
            }
            robot.motorFl.setVelocity(0);
            robot.motorBl.setVelocity(0);
            robot.motorBr.setVelocity(0);
            robot.motorFr.setVelocity(0);

            resetEncoders();

        }
    }

    /*public void moveLift (int position){
        if (opModeIsActive()){
            robot.slider.setTargetPosition(position);
            robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (position > robot.slider.getCurrentPosition()) {
                //robot.cage.setPosition(constant.up);
                robot.slider.setPower(constant.raiseSlider);
            }
            else
                robot.slider.setPower(constant.lowerSlider);
            while (robot.slider.isBusy()){
                telemetry.addData("Moving", "slider...");
                telemetry.addData("Target position: ", "%2d", robot.slider.getTargetPosition());
                telemetry.addData("Current position: ", "%2d", robot.slider.getCurrentPosition());
                telemetry.update();
            }
            robot.slider.setPower(0);
            robot.slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }*/

    public void resetEncoders(){
        robot.motorFl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveArm(double position){
        if (Math.abs(Hardware.leftArm.getPosition() - position) > 0.01){
            movingArm = true;
            if (Hardware.leftArm.getPosition() > position){
                leftPos-=0.03*Math.abs(Hardware.leftArm.getPosition() - position);
                rightPos+=0.03*Math.abs(Hardware.leftArm.getPosition() - position);
            }
            else {
                leftPos+=0.02*Math.abs(Hardware.leftArm.getPosition() - position);
                rightPos-=0.02*Math.abs(Hardware.leftArm.getPosition() - position);
            }
        }
        else movingArm = false;
        if (position > 0.8)
            Hardware.rotateClaw.setPosition(Constant.upsideDownCone);
        else Hardware.rotateClaw.setPosition(Constant.verticalCone);
        if (position > 0.45)
            clawPos = Constant.dropAngle + Math.abs(Constant.leftArmCollect - position);
        else clawPos = Constant.out + Math.abs(Constant.leftArmCollect - position);
    }



}