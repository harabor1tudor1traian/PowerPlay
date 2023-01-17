
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
        position1,
        position2,
        preLoad,
        cone1,
        dropCone1,
        cone2,
        dropCone2,
        cone3,
        dropCone3,
        cone4,
        dropCone4,
        cone5,
        dropCone5,
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
    private ElapsedTime clawTime = new ElapsedTime();
    private Constant ct = new Constant();
    private int parkZone = 2;
    boolean movingArm = false;
    boolean cycleTimer = true;
    boolean cycle4 = false;
    boolean cycle5 = false;
    boolean tagFound = false;
    boolean movingSliders = false;
    boolean dropping = true;
    boolean lowering1 = false;
    boolean lowering2 = false;
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
    double tagsize = 0.45;

    int id1 = 13;
    int id2 = 19;
    int id3 = 12;

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
        Trajectory position1, position2;
        Trajectory park = null;

        position1 = drive.trajectoryBuilder(new Pose2d(), true)
                .lineToConstantHeading(new Vector2d(toInches(-96), toInches(-40)))
                .build();

        position2 = drive.trajectoryBuilder(new Pose2d(), false)
                .splineTo(new Vector2d(toInches(-83),toInches(-50)), -0.19)
                .build();



        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                robot.motorFl.getCurrentPosition(),
                robot.motorBl.getCurrentPosition(),
                robot.motorBr.getCurrentPosition(),
                robot.motorFr.getCurrentPosition());
        telemetry.update();

        waitForStart();
        detectionTime.reset();

        while (opModeIsActive()){

            if (!lowering1 && !lowering2) {
                stabilizeSliders();
                checkToStopSliders();
            }
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
                    drive.followTrajectoryAsync(position1);
                    break;
                case position1:
                    if (drive.isBusy())
                        programstage = thisStage();
                    else{
                        drive.followTrajectoryAsync(position2);
                        programstage = nextStage();
                    }
                    break;
                case position2:
                    moveSliders(Constant.mid);
                    if (drive.isBusy())
                        programstage = thisStage();
                    else {
                        programstage = nextStage();
                    }
                    break;
                case preLoad:
                    moveArm(Constant.stackCone1);
                    Hardware.intake.setPosition(Constant.autonomousStack);
                    Hardware.mainClaw.setPosition(Constant.openClaw);
                    if (dropping) {
                        clawTime.reset();
                        dropping = false;
                    }
                    if (movingArm)
                        programstage = thisStage();
                    else if (clawTime.seconds() > 0.5 && !dropping){
                        Hardware.intakeClaw.setPosition(Constant.closedClaw);
                        sleep(500);
                        programstage = nextStage();
                    }
                    break;

                case cone1:
                    lowerSliders();
                    Hardware.intake.setPosition(Constant.retract);
                    moveArm(Constant.armIn);
                    if (movingArm)
                        programstage = thisStage();
                    else {
                        Hardware.intakeClaw.setPosition(Constant.openIntake);
                        sleep(500);
                        Hardware.mainClaw.setPosition(Constant.closedClaw);
                        programstage = nextStage();
                    }
                    break;
                case dropCone1:
                    moveArm(Constant.stackCone2);
                    Hardware.intake.setPosition(Constant.autonomousStack);
                    moveSliders(Constant.mid);
                    if (movingArm || movingSliders)
                        programstage = thisStage();
                    else {
                        Hardware.mainClaw.setPosition(Constant.openClaw);
                        sleep(500);
                        Hardware.intakeClaw.setPosition(Constant.closedIntake);
                        programstage = nextStage();
                    }
                    break;
                case cone2:
                    lowerSliders();
                    Hardware.intake.setPosition(Constant.retract);
                    moveArm(Constant.armIn);
                    if (movingArm)
                        programstage = thisStage();
                    else {
                        Hardware.intakeClaw.setPosition(Constant.openIntake);
                        sleep(500);
                        Hardware.mainClaw.setPosition(Constant.closedClaw);
                        programstage = nextStage();
                    }
                    break;
                case dropCone2:
                    moveArm(Constant.stackCone3);
                    Hardware.intake.setPosition(Constant.autonomousStack);
                    moveSliders(Constant.mid);
                    if (movingArm || movingSliders)
                        programstage = thisStage();
                    else {
                        Hardware.mainClaw.setPosition(Constant.openClaw);
                        sleep(500);
                        Hardware.intakeClaw.setPosition(Constant.closedIntake);
                        programstage = nextStage();
                    }
                    break;
                case cone3:
                    lowerSliders();
                    Hardware.intake.setPosition(Constant.retract);
                    moveArm(Constant.armIn);
                    if (movingArm)
                        programstage = thisStage();
                    else {
                        Hardware.intakeClaw.setPosition(Constant.openIntake);
                        sleep(500);
                        Hardware.mainClaw.setPosition(Constant.closedClaw);
                        programstage = nextStage();
                    }
                    break;
                case dropCone3:
                    moveArm(Constant.stackCone4);
                    Hardware.intake.setPosition(Constant.autonomousStack);
                    moveSliders(Constant.mid);
                    if (movingArm || movingSliders)
                        programstage = thisStage();
                    else {
                        Hardware.mainClaw.setPosition(Constant.openClaw);
                        sleep(500);
                        Hardware.intakeClaw.setPosition(Constant.closedIntake);
                        programstage = nextStage();
                    }
                    break;
                case cone4:
                    lowerSliders();
                    Hardware.intake.setPosition(Constant.retract);
                    moveArm(Constant.armIn);
                    if (movingArm)
                        programstage = thisStage();
                    else {
                        Hardware.intakeClaw.setPosition(Constant.openIntake);
                        sleep(500);
                        Hardware.mainClaw.setPosition(Constant.closedClaw);
                        programstage = nextStage();
                    }
                    break;
                case dropCone4:
                    moveArm(Constant.stackCone5);
                    Hardware.intake.setPosition(Constant.autonomousStack);
                    moveSliders(Constant.mid);
                    if (movingArm || movingSliders)
                        programstage = thisStage();
                    else {
                        Hardware.mainClaw.setPosition(Constant.openClaw);
                        sleep(500);
                        Hardware.intakeClaw.setPosition(Constant.closedIntake);
                        programstage = nextStage();
                    }
                    break;
                case cone5:
                    lowerSliders();
                    Hardware.intake.setPosition(Constant.retract);
                    moveArm(Constant.armIn);
                    if (movingArm)
                        programstage = thisStage();
                    else {
                        Hardware.intakeClaw.setPosition(Constant.openIntake);
                        sleep(500);
                        Hardware.mainClaw.setPosition(Constant.closedClaw);
                        programstage = nextStage();
                    }
                    break;
                case dropCone5:
                    moveSliders(Constant.mid);
                    if (movingArm)
                        programstage = thisStage();
                    else {
                        Hardware.mainClaw.setPosition(Constant.openClaw);
                        sleep(500);
                        drive.followTrajectoryAsync(park);
                        programstage = nextStage();
                    }
                    break;
                case park:
                    lowerSliders();
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

    public double voltageToDegrees(double voltage){
        return (voltage * Constant.potentiometerMaxDegrees)/Constant.potentiometerMaxVoltage;
    }

    public double degreesToVoltage(double degrees){
        return (degrees * Constant.potentiometerMaxVoltage)/Constant.potentiometerMaxDegrees;
    }

    public double voltageToServo(double voltage){
        return voltageToDegrees(voltage)/300.0;
    }

    public double degreesToServo(double degrees){
        return degrees/300;
    }

    public void moveArm(double position) {
        if (Math.abs(voltageToDegrees(Hardware.potentiometer.getVoltage()) - position) > 3)
            if (voltageToDegrees(Hardware.potentiometer.getVoltage()) > position)
                Hardware.intakeArm.setPower(-Constant.armPower * (voltageToDegrees(Hardware.potentiometer.getVoltage()) / position));
            else if (voltageToDegrees(Hardware.potentiometer.getVoltage()) < position)
                Hardware.intakeArm.setPower(Constant.armPower * (voltageToDegrees(Hardware.potentiometer.getVoltage()) / position));
            else
            if (voltageToDegrees(Hardware.potentiometer.getVoltage()) > position)
                Hardware.intakeArm.setPower(-Constant.stabilizeArm);
            else if (voltageToDegrees(Hardware.potentiometer.getVoltage()) < position)
                Hardware.intakeArm.setPower(Constant.stabilizeArm);
    }

    public static double toInches(double dist){
        return dist/2.54;
    }

    public void moveSliders(int position){
        movingSliders = true;
        Hardware.slider2.setTargetPosition(position);
        if (Hardware.slider2.getCurrentPosition() >= Hardware.slider2.getTargetPosition()) {
            Hardware.slider1.setPower(Constant.lowerSlider);
            Hardware.slider2.setPower(Constant.lowerSlider);
        }
        else if (Hardware.slider2.getCurrentPosition() <= Hardware.slider2.getTargetPosition()) {
            Hardware.slider1.setPower(Constant.raiseSlider);
            Hardware.slider2.setPower(Constant.raiseSlider);
        }
        clawPos = Constant.dropAngle;
    }

    public void checkToStopSliders(){
        if (movingSliders){
            if (Math.abs(Hardware.slider2.getCurrentPosition() - Hardware.slider2.getTargetPosition()) < 10){
                Hardware.slider1.setPower(0.0);
                Hardware.slider2.setPower(0.0);
                movingSliders = false;
            }
        }
    }

    public void lowerSliders() {
        if (Hardware.limitSwitch1.getState()) {
            lowering1 = true;
            Hardware.slider1.setPower(Constant.lowerSlider);
        }
        else {
            lowering1 = false;
            Hardware.slider1.setPower(0.0);
        }
        if (Hardware.limitSwitch2.getState()) {
            lowering2 = true;
            Hardware.slider2.setPower(Constant.lowerSlider);
        }
        else {
            lowering2 = false;
            Hardware.slider2.setPower(0.0);
        }
        Hardware.inOutAngle.setPosition(Constant.in);
    }

    public void stabilizeSliders(){
        int target = 0;
        if (Hardware.slider2.getPower() == 0.0)
            target = Hardware.slider2.getCurrentPosition();
        if (Hardware.slider2.getCurrentPosition() < target && Math.abs(Hardware.slider2.getCurrentPosition()-target) > 1 && Math.abs(Hardware.slider2.getCurrentPosition()-target) < 10){
            Hardware.slider1.setPower(Constant.stopSlider);
            Hardware.slider2.setPower(Constant.stopSlider);
        }
        else if (Hardware.slider2.getCurrentPosition() > target && Math.abs(Hardware.slider2.getCurrentPosition()-target) > 1 && Math.abs(Hardware.slider2.getCurrentPosition()-target) < 10){
            Hardware.slider1.setPower(-Constant.stopSlider);
            Hardware.slider2.setPower(-Constant.stopSlider);
        }
    }

}