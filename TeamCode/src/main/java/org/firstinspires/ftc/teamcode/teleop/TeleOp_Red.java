package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constant;
import org.firstinspires.ftc.teamcode.hardware.Hardware;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp_Red", group="Demo")

public class TeleOp_Red extends LinearOpMode {
    Hardware robot = new Hardware();
    private final ElapsedTime clawDelay = new ElapsedTime();
    private final ElapsedTime clawSensorDelay = new ElapsedTime();
    private final ElapsedTime rotateDelay = new ElapsedTime();
    private final ElapsedTime angleTime = new ElapsedTime();
    private final ElapsedTime changeSideTime = new ElapsedTime();
    private final ElapsedTime matchTime = new ElapsedTime();
    double flspeed;
    double blspeed;
    double brspeed;
    double frspeed;

    boolean clClaw = false;
    boolean opClaw = false;
    boolean vertical = false;
    boolean upsideDown = false;
    boolean in = false;
    boolean movingArm = false;
    boolean back = true;
    boolean vibrate1 = true;
    boolean vibrate2 = true;
    boolean vibrate3 = true;

    double position = 0.0;

    double Drive = 0.0;
    double Turn = 0.0;
    double Slide = 0.0;
    double max = 0.0;
    double leftPos = Constant.leftArmCollect, rightPos = Constant.rightArmCollect;
    double clawPos = Constant.in;

    double brake = 1.0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        checkServos();

        waitForStart();
        clawDelay.reset();
        rotateDelay.reset();
        angleTime.reset();
        changeSideTime.reset();
        matchTime.reset();
        while (opModeIsActive()) {
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

            //GAMEPAD 2

            //Slider

            if (gamepad2.y && changeSideTime.seconds() > 0.2){
                back = !back;
                changeSideTime.reset();
            }

            if (back){
                gamepad1.setLedColor(255, 0, 4, 1000000);
                gamepad2.setLedColor(255, 0, 4, 1000000);
            }
            else{
                gamepad1.setLedColor(10, 255, 0, 1000000);
                gamepad2.setLedColor(10, 255, 0, 1000000);
            }

            if (gamepad2.dpad_up) {
                if (back) {
                    moveSliders(Constant.highBack);
                    position = Constant.armDropHigh;
                }
                else {
                    moveSliders(Constant.highBack);
                    position = Constant.armDropHighBack;
                }
                movingArm = true;
            }
            if (gamepad2.dpad_left) {
                if (back) {
                    moveSliders(Constant.midBack);
                    position = Constant.armDropMidBack;
                }
                else {
                    moveSliders(Constant.mid);
                    position = Constant.armDropMid;
                }
                movingArm = true;
            }
            if (gamepad2.dpad_right){
                moveSliders(Constant.low);
                position = Constant.armDropLow;
                movingArm = true;
            }
            if (gamepad2.dpad_down) {
                Hardware.slider1.setPower(0.0);
                Hardware.slider2.setPower(0.0);
                Hardware.slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware.slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware.slider1.setPower(Constant.lowerSlider);
                Hardware.slider2.setPower(Constant.lowerSlider);
                position = Constant.leftArmCollect;
                movingArm = true;
            }
            /*
            if (-gamepad2.left_stick_y > 0) {
                Hardware.slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware.slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware.slider1.setPower(Constant.raiseSlider * slowButton);
                Hardware.slider1.setPower(Constant.raiseSlider * slowButton);
                manual = true;
            }
            else if (-gamepad2.left_stick_y < 0) {
                Hardware.slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware.slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware.slider1.setPower(Constant.lowerSlider * slowButton);
                Hardware.slider2.setPower(Constant.lowerSlider * slowButton);
                manual = true;
            }
            else if (manual) {
                if (Hardware.limitSwitch1.getState())
                    Hardware.slider1.setPower(Constant.stopSlider);
                if (Hardware.limitSwitch2.getState())
                    Hardware.slider2.setPower(Constant.stopSlider);
                encoderReset(Hardware.slider1, Hardware.limitSwitch1);
                encoderReset(Hardware.slider2, Hardware.limitSwitch2);
                Hardware.slider1.setPower(0.0);
                Hardware.slider2.setPower(0.0);
            }*/

            encoderReset(Hardware.slider1, Hardware.limitSwitch1);
            encoderReset(Hardware.slider2, Hardware.limitSwitch2);
            stabilizeSliders();

            //Claw
            if (gamepad2.a && clawDelay.milliseconds() > 200) {
                if (clClaw)
                    Hardware.mainClaw.setPosition(Constant.openClaw);
                else
                    Hardware.mainClaw.setPosition(Constant.closedClaw);
                clawSensorDelay.reset();
                clawDelay.reset();
            }


            if (gamepad2.b && rotateDelay.milliseconds() > 200) {
                if (upsideDown)
                    Hardware.rotateClaw.setPosition(Constant.verticalCone);
                else
                    Hardware.rotateClaw.setPosition(Constant.upsideDownCone);
                rotateDelay.reset();
            }

            if (gamepad2.x && angleTime.milliseconds() > 200 && !movingArm) {
                if (!in)
                    clawPos = Constant.in;
                else
                    clawPos = Constant.out;
                angleTime.reset();
            }


            if (angleTime.seconds() > 0.7)
                if (vertical)
                    Hardware.rotateClaw.setPosition(Constant.upsideDownCone);
                else
                    Hardware.rotateClaw.setPosition(Constant.verticalCone);

            if (-gamepad2.right_stick_y > 0.5)
                clawPos+=0.005;
            else if (-gamepad2.right_stick_y < -0.5)
                clawPos-=0.005;

            if (Hardware.clawSensor.getDistance(DistanceUnit.CM) < 3 && red())
                Hardware.mainClaw.setPosition(Constant.closedClaw);

            //Arm
            if (-gamepad2.left_stick_y > 0.5){
                leftPos+=0.002;
                rightPos-=0.002;
                if (Hardware.leftArm.getPosition() > Constant.angleUsageThreshold)
                    clawPos = Constant.dropAngle - Math.abs(Hardware.leftArm.getPosition() - Constant.leftArmCollect);
                else clawPos = Constant.out - Math.abs(Hardware.leftArm.getPosition() - Constant.leftArmCollect);
                movingArm = false;
            }
            if (-gamepad2.left_stick_y < -0.5){
                leftPos-=0.001;
                rightPos+=0.001;
                if (Hardware.leftArm.getPosition() > Constant.angleUsageThreshold)
                    clawPos = Constant.dropAngle - Math.abs(Hardware.leftArm.getPosition() - Constant.leftArmCollect);
                else clawPos = Constant.out - Math.abs(Hardware.leftArm.getPosition() - Constant.leftArmCollect);
                movingArm = false;
            }

            if (movingArm)
                moveArm(position);

            Hardware.leftArm.setPosition(leftPos);
            Hardware.rightArm.setPosition(rightPos);
            Hardware.inOutAngle.setPosition(clawPos);

            if (vibrate1 && matchTime.seconds() > 108) {
                gamepad1.rumble(1000);
                vibrate1 = !vibrate1;
            }

            if (vibrate2 && matchTime.seconds() > 113) {
                gamepad1.rumble(1000);
                vibrate2 = !vibrate2;
            }
            if (vibrate3 && matchTime.seconds() > 118) {
                gamepad1.rumble(1000);
                vibrate3 = !vibrate3;
            }

            telemetry.addData("Lift1:", Hardware.slider1.getCurrentPosition());
            telemetry.addData("Lift2:", Hardware.slider2.getCurrentPosition());
            telemetry.addData("Switch1:", Hardware.limitSwitch1.getState());
            telemetry.addData("Switch2:", Hardware.limitSwitch2.getState());
            telemetry.update();
        }
    }

    public void moveSliders(int position){
        Hardware.slider1.setTargetPosition(position);
        Hardware.slider2.setTargetPosition(position);
        Hardware.slider1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Hardware.slider2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (Hardware.slider1.getCurrentPosition() >= Hardware.slider1.getTargetPosition()) {
            Hardware.slider1.setPower(Constant.lowerSlider);
            Hardware.slider2.setPower(Constant.lowerSlider);
        }
        else if (Hardware.slider1.getCurrentPosition() <= Hardware.slider1.getTargetPosition()) {
            Hardware.slider1.setPower(Constant.raiseSlider);
            Hardware.slider2.setPower(Constant.raiseSlider);
        }
    }

    public void stabilizeSliders(){
        if (Hardware.slider1.getCurrentPosition() < Hardware.slider1.getTargetPosition() && Math.abs(Hardware.slider1.getCurrentPosition() - Hardware.slider1.getTargetPosition())<10 && Math.abs(Hardware.slider1.getCurrentPosition() - Hardware.slider1.getTargetPosition())>1 && Hardware.slider1.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
            Hardware.slider1.setPower(Constant.stopSlider);
        else if (Hardware.slider1.getCurrentPosition() > Hardware.slider1.getTargetPosition() && Math.abs(Hardware.slider1.getCurrentPosition() - Hardware.slider1.getTargetPosition())<10 && Math.abs(Hardware.slider1.getCurrentPosition() - Hardware.slider1.getTargetPosition())>1 && Hardware.slider1.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
            Hardware.slider1.setPower(-Constant.stopSlider);

        if (Hardware.slider2.getCurrentPosition() < Hardware.slider2.getTargetPosition() && Math.abs(Hardware.slider2.getCurrentPosition() - Hardware.slider2.getTargetPosition())<10 && Math.abs(Hardware.slider2.getCurrentPosition() - Hardware.slider2.getTargetPosition())>1 && Hardware.slider2.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
            Hardware.slider2.setPower(Constant.stopSlider);
        else if (Hardware.slider2.getCurrentPosition() > Hardware.slider2.getTargetPosition() && Math.abs(Hardware.slider2.getCurrentPosition() - Hardware.slider2.getTargetPosition())<10 && Math.abs(Hardware.slider2.getCurrentPosition() - Hardware.slider2.getTargetPosition())>1 && Hardware.slider2.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
            Hardware.slider2.setPower(-Constant.stopSlider);
    }

    public void encoderReset(DcMotor slider, DigitalChannel limitSwitch){
        if (slider.getPower() < 0.0 && !limitSwitch.getState()) {
            slider.setPower(0.0);
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void checkServos(){
        if (Math.abs(Hardware.mainClaw.getPosition() - Constant.closedClaw) < 0.0001)
            clClaw = true;
        else clClaw = false;
        if (Math.abs(Hardware.mainClaw.getPosition() - Constant.openClaw) < 0.0001)
            opClaw = true;
        else opClaw = false;

        if (Math.abs(Hardware.rotateClaw.getPosition() - Constant.verticalCone) < 0.0001)
            vertical = true;
        else vertical = false;

        if (Math.abs(Hardware.rotateClaw.getPosition() - Constant.upsideDownCone) < 0.0001)
            upsideDown = true;
        else upsideDown = false;

        if (Math.abs(Hardware.inOutAngle.getPosition() - Constant.in) < 0.0001)
            in = true;
        else in = false;
    }

    public boolean red(){
        if (Math.max(Math.max(Hardware.clawSensor.blue(), Hardware.clawSensor.red()), Hardware.clawSensor.green()) == Hardware.clawSensor.red())
            return true;
        else return false;
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
