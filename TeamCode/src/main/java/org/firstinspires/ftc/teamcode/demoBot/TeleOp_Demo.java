package org.firstinspires.ftc.teamcode.demoBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constant;
import org.firstinspires.ftc.teamcode.demoBot.Hardware_Demo;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp_Demo", group="Demo")

public class TeleOp_Demo extends LinearOpMode {
    Hardware_Demo robot = new Hardware_Demo();
      private ElapsedTime bormasinaOn = new ElapsedTime();
      private ElapsedTime bormasinatime = new ElapsedTime();
    private ElapsedTime knifeSwing = new ElapsedTime();
    double flspeed;
    double blspeed;
    double brspeed;
    double frspeed;
    boolean start = true;
    boolean running = false;
    boolean manualAngle = false;

    double Drive = 0;
    double Turn = 0;
    double Slide = 0;
    double max;

    double  brake = 1.0;

    boolean manual = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
        bormasinatime.reset();
        knifeSwing.reset();
        while (opModeIsActive()) {
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

            Hardware_Demo.motorFl.setPower(flspeed * brake);
            Hardware_Demo.motorBl.setPower(blspeed * brake);
            Hardware_Demo.motorBr.setPower(brspeed * brake);
            Hardware_Demo.motorFr.setPower(frspeed * brake);

            //GAMEPAD 2

            //Slider
            if (gamepad2.dpad_up) {
                Hardware_Demo.slider.setTargetPosition(constant.high);
                Hardware_Demo.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Hardware_Demo.slider.setPower(constant.raiseSlider);
                manual = false;
                manualAngle = false;
            }
            if (gamepad2.dpad_left) {
                Hardware_Demo.slider.setTargetPosition(constant.mid);
                Hardware_Demo.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (Hardware_Demo.slider.getCurrentPosition() >= Hardware_Demo.slider.getTargetPosition())
                    Hardware_Demo.slider.setPower(constant.lowerSlider);
                else if (Hardware_Demo.slider.getCurrentPosition() <= Hardware_Demo.slider.getTargetPosition())
                    Hardware_Demo.slider.setPower(constant.raiseSlider);
                manual = false;
                manualAngle = false;
            }
            if (gamepad2.dpad_right){
                Hardware_Demo.slider.setTargetPosition(constant.low);
                Hardware_Demo.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (Hardware_Demo.slider.getCurrentPosition() >= Hardware_Demo.slider.getTargetPosition())
                    Hardware_Demo.slider.setPower(constant.lowerSlider);
                else if (Hardware_Demo.slider.getCurrentPosition() <= Hardware_Demo.slider.getTargetPosition())
                    Hardware_Demo.slider.setPower(constant.raiseSlider);
                manual = false;
                manualAngle = false;
            }
            if (gamepad2.dpad_down) {
                Hardware_Demo.slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware_Demo.slider.setPower(0.0);
                Hardware_Demo.slider.setPower(constant.lowerSlider);
                manual = false;
                manualAngle = false;
            }

            if (Hardware_Demo.slider.getPower() < 0.0 && !Hardware_Demo.limitSwitch.getState()
            ) {
                Hardware_Demo.slider.setPower(0.0);
                Hardware_Demo.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Hardware_Demo.slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (Hardware_Demo.slider.getCurrentPosition() < Hardware_Demo.slider.getTargetPosition() && Math.abs(Hardware_Demo.slider.getCurrentPosition() - Hardware_Demo.slider.getTargetPosition())<10 && Math.abs(Hardware_Demo.slider.getCurrentPosition() - Hardware_Demo.slider.getTargetPosition())>1 && Hardware_Demo.slider.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                Hardware_Demo.slider.setPower(constant.stopSlider);
            }
            else if (Hardware_Demo.slider.getCurrentPosition() > Hardware_Demo.slider.getTargetPosition() && Math.abs(Hardware_Demo.slider.getCurrentPosition() - Hardware_Demo.slider.getTargetPosition())<10 && Math.abs(Hardware_Demo.slider.getCurrentPosition() - Hardware_Demo.slider.getTargetPosition())>1 && Hardware_Demo.slider.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                Hardware_Demo.slider.setPower(-constant.stopSlider);
            }

            if (-gamepad2.left_stick_y > 0) {
                Hardware_Demo.slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware_Demo.slider.setPower(-gamepad2.left_stick_y);
                manual = true;
            }
            else if (-gamepad2.left_stick_y < 0) {
                Hardware_Demo.slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware_Demo.slider.setPower(-gamepad2.left_stick_y);
                manual = true;
            }
            else if (Hardware_Demo.limitSwitch.getState() && manual) Hardware_Demo.slider.setPower(constant.stopSlider);
            else if (manual) Hardware_Demo.slider.setPower(0.0);
            //claw
            /*if (gamepad2.b){
                Hardware_Demo.claw.setPosition(constant.openClaw);
            }
            else if (gamepad2.a){
                Hardware_Demo.claw.setPosition(constant.closedClaw);
            }*/
            /*if (gamepad2.x){
                manualAngle = true;
                Hardware_Demo.angle.setPosition(constant.collectAngle);
            }

            if (gamepad2.y){
                manualAngle = true;
                Hardware_Demo.angle.setPosition(constant.dropAngle);
            }

            if (Hardware_Demo.slider.getCurrentPosition() > 500 && !manualAngle){
                Hardware_Demo.angle.setPosition(constant.dropAngle);
            }
            if (Hardware_Demo.slider.getCurrentPosition() < 500 && !manualAngle){
                Hardware_Demo.angle.setPosition(constant.collectAngle);
            }*/


            telemetry.addData("Lift:", Hardware_Demo.slider.getCurrentPosition());
            telemetry.addData("Lift:", Hardware_Demo.slider.getCurrentPosition());
            //telemetry.addData("Voltage:", Hardware_Demo.expansionHub.getInputVoltage(VoltageUnit.VOLTS));
            telemetry.addData("Switch:", Hardware_Demo.limitSwitch.getState());
            telemetry.update();
        }
    }
}
