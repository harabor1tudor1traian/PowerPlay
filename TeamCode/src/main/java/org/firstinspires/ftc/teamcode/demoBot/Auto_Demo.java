
package org.firstinspires.ftc.teamcode.demoBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constant;
import org.firstinspires.ftc.teamcode.vision.ElementDetection;

@Autonomous(name="Autonomous_Right", group="Blue_encoder")
public class Auto_Demo extends LinearOpMode {

    Hardware_Demo robot = new Hardware_Demo();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime detectionTime = new ElapsedTime();
    private Constant ct = new Constant();
    private int level = 2;
    private boolean firsttime = true;
    private int distanceBack = 0;
    private double voltage;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.motorFl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElementDetection detectElement = new ElementDetection(hardwareMap, telemetry);

        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                robot.motorFl.getCurrentPosition(),
                robot.motorBl.getCurrentPosition(),
                robot.motorBr.getCurrentPosition(),
                robot.motorFr.getCurrentPosition());
        telemetry.update();

        waitForStart();
        runtime.reset();
        //voltage = Hardware_Demo.expansionHub.getInputVoltage(VoltageUnit.VOLTS);


        /*detectionTime.reset();
        while (detectionTime.seconds() < 0.3) {
            if (detectElement.randomisation == 1)
                level = 1;
            else if (detectElement.randomisation == 2)
                level = 2;
            else if (detectElement.randomisation == 3)
                level = 3;
            else level = 2;
        }*/
        driveForward(120,0,0);
        telemetry.addData("Path", "Complete");
        telemetry.addData("Randomisation:", level);
        telemetry.update();
        sleep(20000);
    }

    public void driveForward(double distance, int leftDif, int rightDif) {
        if (opModeIsActive()) {
            int flTarget, blTarget, brTarget, frTarget;
            firsttime = true;
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
               /* if (((Math.abs(robot.motorFl.getCurrentPosition() - robot.motorFl.getTargetPosition()) < (ct.ticksPerRev * 5) / (ct.wheelDiameter * 3.1415))
                         || (Math.abs(robot.motorBl.getCurrentPosition() - robot.motorBl.getTargetPosition()) < (ct.ticksPerRev * 5) / (ct.wheelDiameter * 3.1415))
                         || (Math.abs(robot.motorBr.getCurrentPosition() - robot.motorBr.getTargetPosition()) < (ct.ticksPerRev * 5) / (ct.wheelDiameter * 3.1415))
                         || (Math.abs(robot.motorFr.getCurrentPosition() - robot.motorFr.getTargetPosition()) < (ct.ticksPerRev * 5) / (ct.wheelDiameter * 3.1415))) && firsttime) {
                    for (int i = ct.maxVelocityDrivetrain; i >= 0; i-=100) {
                        robot.motorFl.setVelocity(i, AngleUnit.DEGREES);
                        robot.motorBl.setVelocity(i, AngleUnit.DEGREES);
                        robot.motorBr.setVelocity(i, AngleUnit.DEGREES);
                        robot.motorFr.setVelocity(i, AngleUnit.DEGREES);
                        sleep(100);
                    }
                    firsttime = false;
                }*/
            }

            robot.motorFl.setVelocity(0, AngleUnit.DEGREES);
            robot.motorBl.setVelocity(0, AngleUnit.DEGREES);
            robot.motorBr.setVelocity(0, AngleUnit.DEGREES);
            robot.motorFr.setVelocity(0, AngleUnit.DEGREES);

            robot.motorFl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.motorFl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

    public void driveBackward(double distance, int leftDif, int rightDif) {
        if (opModeIsActive()) {
            int flTarget, blTarget, brTarget, frTarget;
            firsttime=true;
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

            robot.motorFl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.motorFl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void slideRight(double distance) {
        if (opModeIsActive()) {
            int flTarget, blTarget, brTarget, frTarget;
            firsttime = true;
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

            robot.motorFl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.motorFl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }


    public void slideLeft(double distance) {
        if (opModeIsActive()) {
            int flTarget, blTarget, brTarget, frTarget;
            firsttime = true;
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
                /*if (((Math.abs(robot.motorFl.getCurrentPosition() - robot.motorFl.getTargetPosition()) < (constant.ticksPerRev * 5) / (constant.wheelDiameter * 3.1415))
                         || (Math.abs(robot.motorBl.getCurrentPosition() - robot.motorBl.getTargetPosition()) < (constant.ticksPerRev * 5) / (constant.wheelDiameter * 3.1415))
                         || (Math.abs(robot.motorBr.getCurrentPosition() - robot.motorBr.getTargetPosition()) < (constant.ticksPerRev * 5) / (constant.wheelDiameter * 3.1415))
                         || (Math.abs(robot.motorFr.getCurrentPosition() - robot.motorFr.getTargetPosition()) < (constant.ticksPerRev * 5) / (constant.wheelDiameter * 3.1415))) && firsttime) {
                    for (int i = constant.maxVelocityDrivetrain/2; i >= 0; i-=100) {
                        robot.motorFl.setVelocity(-i, AngleUnit.DEGREES);
                        robot.motorBl.setVelocity(i, AngleUnit.DEGREES);
                        robot.motorBr.setVelocity(-i, AngleUnit.DEGREES);
                        robot.motorFr.setVelocity(i, AngleUnit.DEGREES);
                        sleep(100);
                    }
                    firsttime = false;
                }*/
            }

            robot.motorFl.setVelocity(0, AngleUnit.DEGREES);
            robot.motorBl.setVelocity(0, AngleUnit.DEGREES);
            robot.motorBr.setVelocity(0, AngleUnit.DEGREES);
            robot.motorFr.setVelocity(0, AngleUnit.DEGREES);

            robot.motorFl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.motorFl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

            robot.motorFl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.motorFl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

            robot.motorFl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.motorFl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void moveLift (int position){
        if (opModeIsActive()){
            robot.slider.setTargetPosition(position);
            robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (position > robot.slider.getCurrentPosition()) {
                //robot.cage.setPosition(constant.up);
                robot.slider.setPower(Constant.raiseSlider);
            }
            else
                robot.slider.setPower(Constant.lowerSlider);
            while (robot.slider.isBusy()){
                telemetry.addData("Moving", "slider...");
                telemetry.addData("Target position: ", "%2d", robot.slider.getTargetPosition());
                telemetry.addData("Current position: ", "%2d", robot.slider.getCurrentPosition());
                telemetry.update();
            }
            robot.slider.setPower(0);
            robot.slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


}
