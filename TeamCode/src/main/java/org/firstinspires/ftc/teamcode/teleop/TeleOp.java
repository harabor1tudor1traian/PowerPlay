package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constant;
import org.firstinspires.ftc.teamcode.hardware.Hardware;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Demo")

public class TeleOp extends LinearOpMode {
    Hardware robot = new Hardware();
    private ElapsedTime clawDelay = new ElapsedTime();
    private ElapsedTime rotateDelay = new ElapsedTime();
    private ElapsedTime cycleTime = new ElapsedTime();
    double flspeed;
    double blspeed;
    double brspeed;
    double frspeed;
    boolean start = true;
    boolean running = false;
    boolean clClaw = false;
    boolean opClaw = false;
    boolean vertical = false;
    boolean upsideDown = false;
    boolean preset = false;
    boolean drop = false;
    double Drive = 0;
    double Turn = 0;
    double Slide = 0;
    double max;
    double leftPos = 0.5, rightPos = 0.5;

    double brake = 1.0;
    double slowButton = 1.0;

    boolean manual = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        checkServos();

        waitForStart();
        clawDelay.reset();
        rotateDelay.reset();
        while (opModeIsActive()) {
            cycleTime.reset();
            checkServos();
            ///GAMEPAD 1

            //BRAKE
            if (gamepad1.left_trigger > 0.0)
                brake = 0.3;
            else if (gamepad1.right_trigger > 0.0)
                brake = 0.5;
            else
                brake = 1.0;

            //DRIVING
            Drive  = -gamepad1.left_stick_y;
            Slide  =  gamepad1.left_stick_x;
            Turn   =  gamepad1.right_stick_x;

            flspeed = Drive + Slide + Turn;
            blspeed = Drive - Slide + Turn;
            brspeed = Drive + Slide - Turn;
            frspeed = Drive - Slide - Turn;

            max = Math.max(Math.max(Math.abs(flspeed),Math.abs(frspeed)), Math.max(Math.abs(blspeed), Math.abs(brspeed)));

            if (max>1){
                flspeed/=max;
                frspeed/=max;
                blspeed/=max;
                brspeed/=max;
            }

            Hardware.motorFl.setPower(flspeed * brake);
            Hardware.motorBl.setPower(blspeed * brake);
            Hardware.motorBr.setPower(brspeed * brake);
            Hardware.motorFr.setPower(frspeed * brake);

            telemetry.addData("Left:", Hardware.leftEncoder.getCurrentPosition());
            telemetry.addData("Right:", Hardware.rightEncoder.getCurrentPosition());
            telemetry.addData("Front:", Hardware.frontEncoder .getCurrentPosition());
            telemetry.addData("Rotate:", Hardware.rotateClaw.getPosition());
            telemetry.addData("Main:", Hardware.mainClaw.getPosition());
            telemetry.addData("leftArm", Hardware.leftArm.getPosition());
            telemetry.addData("rightArm", Hardware.rightArm.getPosition());
            telemetry.update();

            if (gamepad2.a  && clClaw && clawDelay.milliseconds() > 200){
                Hardware.mainClaw.setPosition(constant.openClaw);
                clawDelay.reset();
            }
            if (gamepad2.a  && opClaw && clawDelay.milliseconds() > 200){
                Hardware.mainClaw.setPosition(constant.closedClaw);
                clawDelay.reset();
            }


            if (gamepad2.b && upsideDown && rotateDelay.milliseconds() > 200){
                Hardware.rotateClaw.setPosition(constant.verticalCone);
                rotateDelay.reset();
            }
            if (gamepad2.b && vertical && rotateDelay.milliseconds() > 200){
                Hardware.rotateClaw.setPosition(constant.upsideDownCone);
                rotateDelay.reset();
            }
            
            if (-gamepad2.left_stick_y > 0.5){
                leftPos+=0.002;
                rightPos-=0.002;
                preset = false;
            }
            if (-gamepad2.left_stick_y < -0.5){
                leftPos-=0.001;
                rightPos+=0.001;
                preset = false;
            }

            if (gamepad2.left_bumper){
                preset = true;
                drop = false;
            }
            if (gamepad2.right_bumper){
                preset = true;
                drop = true;
            }

            if (preset)
                moveArm(drop);

            Hardware.leftArm.setPosition(leftPos);
            Hardware.rightArm.setPosition(rightPos);


            //GAMEPAD 2

            //Slider
            /*if (gamepad2.dpad_up) {
                moveSliders(constant.high);
            }
            if (gamepad2.dpad_left) {
                moveSliders(constant.mid);
            }
            if (gamepad2.dpad_right){
                moveSliders(constant.low);
            }
            if (gamepad2.dpad_down) {
                Hardware.slider1.setPower(0.0);
                Hardware.slider2.setPower(0.0);
                Hardware.slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware.slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware.slider1.setPower(constant.lowerSlider);
                Hardware.slider2.setPower(constant.lowerSlider);
                manual = false;
            }

            encoderReset(Hardware.slider1, Hardware.limitSwitch1);
            encoderReset(Hardware.slider2, Hardware.limitSwitch2);
            stabilizeSliders();

            if (-gamepad2.left_stick_y > 0) {
                Hardware.slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware.slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware.slider1.setPower(constant.raiseSlider * slowButton);
                Hardware.slider1.setPower(constant.raiseSlider * slowButton);
                manual = true;
            }
            else if (-gamepad2.left_stick_y < 0) {
                Hardware.slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware.slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware.slider1.setPower(constant.lowerSlider * slowButton);
                Hardware.slider2.setPower(constant.lowerSlider * slowButton);
                manual = true;
            }
            else if (Hardware.limitSwitch1.getState() && manual) Hardware.slider1.setPower(constant.stopSlider);
            else if (Hardware.limitSwitch2.getState() && manual) Hardware.slider2.setPower(constant.stopSlider);
            else if (manual) {
                encoderReset(Hardware.slider1, Hardware.limitSwitch1);
                encoderReset(Hardware.slider2, Hardware.limitSwitch2);
                Hardware.slider1.setPower(0.0);
                Hardware.slider2.setPower(0.0);
            }
            //claw
            if (gamepad2.b){
                Hardware.mainClaw.setPosition(constant.openClaw);
            }
            else if (gamepad2.a){
                Hardware.mainClaw.setPosition(constant.closedClaw);
            }

            if (Hardware.slider1.getCurrentPosition() > 500 && Hardware.slider2.getCurrentPosition() > 500){
                Hardware.inOutAngle.setPosition(constant.dropAngle);
            }
            if (Hardware.slider1.getCurrentPosition() < 500 && Hardware.slider2.getCurrentPosition() < 500){
                Hardware.inOutAngle.setPosition(constant.out);
            }


            telemetry.addData("Lift1:", Hardware.slider1.getCurrentPosition());
            telemetry.addData("Lift2:", Hardware.slider2.getCurrentPosition());
            telemetry.addData("Switch1:", Hardware.limitSwitch1.getState());
            telemetry.addData("Switch2:", Hardware.limitSwitch2.getState());
            telemetry.update();*/
        }
    }
    
    /*public void moveSliders(int position){
        Hardware.slider1.setTargetPosition(position);
        Hardware.slider2.setTargetPosition(position);
        Hardware.slider1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Hardware.slider2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (Hardware.slider1.getCurrentPosition() >= Hardware.slider1.getTargetPosition()) {
            Hardware.slider1.setPower(constant.lowerSlider);
            Hardware.slider2.setPower(constant.lowerSlider);
        }
        else if (Hardware.slider1.getCurrentPosition() <= Hardware.slider1.getTargetPosition()) {
            Hardware.slider1.setPower(constant.raiseSlider);
            Hardware.slider2.setPower(constant.raiseSlider);
        }
        manual = false;
    }

    public void stabilizeSliders(){
        if (Hardware.slider1.getCurrentPosition() < Hardware.slider1.getTargetPosition() && Math.abs(Hardware.slider1.getCurrentPosition() - Hardware.slider1.getTargetPosition())<10 && Math.abs(Hardware.slider1.getCurrentPosition() - Hardware.slider1.getTargetPosition())>1 && Hardware.slider1.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
            Hardware.slider1.setPower(constant.stopSlider);
        else if (Hardware.slider1.getCurrentPosition() > Hardware.slider1.getTargetPosition() && Math.abs(Hardware.slider1.getCurrentPosition() - Hardware.slider1.getTargetPosition())<10 && Math.abs(Hardware.slider1.getCurrentPosition() - Hardware.slider1.getTargetPosition())>1 && Hardware.slider1.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
            Hardware.slider1.setPower(-constant.stopSlider);

        if (Hardware.slider2.getCurrentPosition() < Hardware.slider2.getTargetPosition() && Math.abs(Hardware.slider2.getCurrentPosition() - Hardware.slider2.getTargetPosition())<10 && Math.abs(Hardware.slider2.getCurrentPosition() - Hardware.slider2.getTargetPosition())>1 && Hardware.slider2.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
            Hardware.slider2.setPower(constant.stopSlider);
        else if (Hardware.slider2.getCurrentPosition() > Hardware.slider2.getTargetPosition() && Math.abs(Hardware.slider2.getCurrentPosition() - Hardware.slider2.getTargetPosition())<10 && Math.abs(Hardware.slider2.getCurrentPosition() - Hardware.slider2.getTargetPosition())>1 && Hardware.slider2.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
            Hardware.slider2.setPower(-constant.stopSlider);

        manual = false;
    }

    public void encoderReset(DcMotor slider, DigitalChannel limitSwitch){
        if (slider.getPower() < 0.0 && limitSwitch.getState()) {
            slider.setPower(0.0);
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }*/

    public void checkServos(){
        if (Math.abs(Hardware.mainClaw.getPosition() - constant.closedClaw) < 0.0001)
            clClaw = true;
        else clClaw = false;
        if (Math.abs(Hardware.mainClaw.getPosition() - constant.openClaw) < 0.0001)
            opClaw = true;
        else opClaw = false;

        if (Math.abs(Hardware.rotateClaw.getPosition() - constant.verticalCone) < 0.0001)
            vertical = true;
        else vertical = false;

        if (Math.abs(Hardware.rotateClaw.getPosition() - constant.upsideDownCone) < 0.0001)
            upsideDown = true;
        else upsideDown = false;
    }

    public void moveArm(boolean drop){
        if (drop && Math.abs(Hardware.leftArm.getPosition() - constant.leftArmDrop) > 0.001){
            if (Hardware.leftArm.getPosition() > constant.leftArmDrop){
                leftPos-=0.03*Math.abs(Hardware.leftArm.getPosition() - constant.leftArmDrop);
                rightPos+=0.03*Math.abs(Hardware.leftArm.getPosition() - constant.leftArmDrop);
            }
            else {
                leftPos+=0.05*Math.abs(Hardware.leftArm.getPosition() - constant.leftArmDrop);
                rightPos-=0.05*Math.abs(Hardware.leftArm.getPosition() - constant.leftArmDrop);
            }
        }
        else if (!drop && Math.abs(Hardware.leftArm.getPosition() - constant.leftArmCollect) > 0.001){
            if (Hardware.leftArm.getPosition() > constant.leftArmCollect){
                leftPos-=0.03*Math.abs(Hardware.leftArm.getPosition() - constant.leftArmCollect);
                rightPos+=0.03*Math.abs(Hardware.leftArm.getPosition() - constant.leftArmCollect);
            }
            else {
                leftPos+=0.05*Math.abs(Hardware.leftArm.getPosition() - constant.leftArmCollect);
                rightPos-=0.05*Math.abs(Hardware.leftArm.getPosition() - constant.leftArmCollect);
            }
        }
        else preset = false;

    }
}
