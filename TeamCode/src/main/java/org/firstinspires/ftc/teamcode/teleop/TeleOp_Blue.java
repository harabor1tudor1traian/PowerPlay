package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constant;
import org.firstinspires.ftc.teamcode.hardware.Hardware;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp_Blue", group="Demo")

public class TeleOp_Blue extends LinearOpMode {
    Hardware robot = new Hardware();
    private final ElapsedTime clawDelay = new ElapsedTime();
    private final ElapsedTime clawSensorDelay = new ElapsedTime();
    private final ElapsedTime rotateDelay = new ElapsedTime();
    private final ElapsedTime angleTime = new ElapsedTime();
    private final ElapsedTime changeSideTime = new ElapsedTime();
    private final ElapsedTime matchTime = new ElapsedTime();
    private final ElapsedTime intakeClawTime = new ElapsedTime();
    private final ElapsedTime intakeDownTime = new ElapsedTime();
    double flspeed;
    double blspeed;
    double brspeed;
    double frspeed;

    boolean clClaw = false;
    boolean opClaw = false;
    boolean vertical = false;
    boolean upsideDown = false;
    boolean in = false;
    boolean vibrate1 = true;
    boolean vibrate2 = true;
    boolean vibrate3 = true;
    boolean lowering = false;
    boolean rotate = false;
    boolean down = false;
    boolean closed = false;
    boolean manual = false;
    boolean preset = false;

    double Drive = 0.0;
    double Turn = 0.0;
    double Slide = 0.0;
    double max = 0.0;
    double slowButton = 0.0;
    double leftPos = Constant.leftArmCollect, rightPos = Constant.rightArmCollect;
    double clawPos = Constant.in;
    double intakePos = Constant.intakeIn;
    double intakeClawPos = Constant.intakeInit;
    double intakeTarget = Constant.intakeIn;

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
        intakeClawTime.reset();
        intakeDownTime.reset();
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
            telemetry.addData("Main:", Hardware.mainClaw.getPosition());
            telemetry.addData("arm", Hardware.intakeArm.getCurrentPosition());

            if (gamepad1.dpad_left)
                intakePos+=0.001;
            if (gamepad1.dpad_right)
                intakePos-=0.001;
            if (gamepad1.right_bumper)
                intakeTarget = intakePos;
            if (gamepad1.left_bumper)
                intakePos = intakeTarget;

            if (gamepad1.dpad_down)
                Hardware.intakeArm.setPower(0.3);
            else if (gamepad1.dpad_up)
                Hardware.intakeArm.setPower(-0.3);
            else if (manual)
                Hardware.intakeArm.setPower(0.0);

            stabilizeArm();
            encoderResetArm(Hardware.intakeArm, Hardware.limitSwitchArm);

            if (gamepad1.y && intakeDownTime.seconds() > 0.2) {
                if (!down) {
                    intakeClawPos = Constant.intakeDown;
                    down = true;
                } else if (down)
                    down = false;
                intakeDownTime.reset();
            }

            if (down && Hardware.intakeSensor.getDistance(DistanceUnit.CM) < 2 && blue(Hardware.intakeSensor) && Hardware.intakeArm.getCurrentPosition() > Constant.angleUsageThreshold)
                Hardware.intakeClaw.setPosition(Constant.closedIntake);

            if (gamepad1.a && intakeClawTime.seconds() > 0.2) {
                if (closed) {
                    closed = false;
                    Hardware.intakeClaw.setPosition(Constant.openIntake);
                } else if (!closed) {
                    closed = true;
                    Hardware.intake.setPosition(Constant.closedIntake);
                }
                intakeClawTime.reset();
            }
            if (Hardware.intakeArm.getCurrentPosition() > Constant.angleUsageThreshold && !down)
                intakeClawPos = Constant.intakeStraight - ticksToServo(Math.abs(Hardware.intakeArm.getCurrentPosition()));

            Hardware.intake.setPosition(intakePos);
            Hardware.moveIntake.setPosition(intakeClawPos);


            //GAMEPAD 2

            //Slider

            if (gamepad2.dpad_up)
                moveSliders(Constant.high);
            if (gamepad2.dpad_left)
                moveSliders(Constant.mid);
            if (gamepad2.dpad_right)
                moveSliders(Constant.low);
            if (gamepad2.dpad_down) {
                Hardware.slider1.setPower(0.0);
                Hardware.slider2.setPower(0.0);
                Hardware.slider1.setPower(Constant.lowerSlider);
                Hardware.slider2.setPower(Constant.lowerSlider);
                preset = true;
                lowering = true;
            }

            if (lowering && Hardware.slider1.getCurrentPosition() < 100){
                Hardware.slider1.setPower(-0.3);
                Hardware.slider2.setPower(-0.3);
            }

            if (Hardware.slider1.getCurrentPosition() > Constant.low - 40 && !preset)
                clawPos = Constant.dropAngle;
            if (Hardware.slider1.getCurrentPosition() < Constant.low - 40 && !preset)
                clawPos = Constant.out;

            if (gamepad2.left_trigger > 0.0)
                slowButton = 0.5;
            else if (gamepad2.right_trigger > 0.0)
                slowButton = 0.7;
            else slowButton = 1.0;

            if (-gamepad2.left_stick_y > 0) {
                Hardware.slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware.slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware.slider1.setPower(Constant.raiseSlider * slowButton);
                Hardware.slider1.setPower(Constant.raiseSlider * slowButton);
                preset = false;
            }
            else if (-gamepad2.left_stick_y < 0) {
                Hardware.slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware.slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware.slider1.setPower(Constant.lowerSlider * slowButton);
                Hardware.slider2.setPower(Constant.lowerSlider * slowButton);
                preset = false;
            }
            else {
                Hardware.slider1.setPower(0.0);
                Hardware.slider2.setPower(0.0);
            }

            encoderReset(Hardware.slider2, Hardware.limitSwitch2);
            checkToStopSliders();
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


            if (gamepad2.x && angleTime.milliseconds() > 200) {
                if (!in)
                    clawPos = Constant.in;
                else
                    clawPos = Constant.out;
                angleTime.reset();
                rotate = true;
            }

            if (-gamepad2.right_stick_y > 0.0)
                clawPos+=0.01;
            else if (-gamepad2.right_stick_y < -0.0)
                clawPos-=0.01;

            if (Hardware.clawSensor.getDistance(DistanceUnit.CM) < 3 && blue(Hardware.clawSensor))
                Hardware.mainClaw.setPosition(Constant.closedClaw);

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
            telemetry.addData("CLawPos:", clawPos);
            telemetry.update();
        }
    }

    public void moveSliders(int position){
        preset = true;
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
        if (preset){
            if (Math.abs(Hardware.slider2.getCurrentPosition() - Hardware.slider2.getTargetPosition()) < 10){
                Hardware.slider1.setPower(0.0);
                Hardware.slider2.setPower(0.0);
                preset = false;
            }
        }
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

    public void stabilizeArm(){
        int target = 0;
        if (Hardware.intakeArm.getPower() == 0.0)
            target = Hardware.intakeArm.getCurrentPosition();
        if (Hardware.intakeArm.getCurrentPosition() < target && Math.abs(Hardware.intakeArm.getCurrentPosition() - target) > 1 && Hardware.intakeArm.getCurrentPosition() - target < 5)
            Hardware.intakeArm.setPower(Constant.stopArm);
        if (Hardware.intakeArm.getCurrentPosition() > target && Math.abs(Hardware.intakeArm.getCurrentPosition() - target) > 1 && Hardware.intakeArm.getCurrentPosition() - target < 5)
            Hardware.intakeArm.setPower(-Constant.stopArm);
    }

    public void encoderReset(DcMotor slider, DigitalChannel limitSwitch){
        if (slider.getPower() < 0.0 && limitSwitch.getState()) {
            slider.setPower(0.0);
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lowering = false;
        }
    }

    public void encoderResetArm(DcMotor motor, DigitalChannel limitSwitch){
        if (motor.getPower() < 0.0 && limitSwitch.getState()) {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void checkServos(){
        if (Math.abs(Hardware.mainClaw.getPosition() - Constant.closedClaw) < 0.0001)
            clClaw = true;
        else clClaw = false;
        if (Math.abs(Hardware.mainClaw.getPosition() - Constant.openClaw) < 0.0001)
            opClaw = true;
        else opClaw = false;

        if (Math.abs(Hardware.inOutAngle.getPosition() - Constant.in) < 0.0001)
            in = true;
        else in = false;
    }

    public boolean blue(RevColorSensorV3 clawSensor){
        if (Math.max(Math.max(clawSensor.blue(), clawSensor.red()), clawSensor.green()) == clawSensor.blue())
            return true;
        else return false;
    }

    public double ticksToServo(double ticks){
        return (ticks/Constant.encoderTicksIntake)*5/6;
    }
}
