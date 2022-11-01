package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.constant;



@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Infinity_TeleOp")
@Disabled
public class TeleOp extends LinearOpMode {

    Hardware robot = new Hardware();
    double flspeed;
    double blspeed;
    double brspeed;
    double frspeed;

    double Drive = 0;
    double Turn = 0;
    double Slide = 0;

    double  brake = 1.0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

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

            Hardware.motorFl.setPower(flspeed * brake);
            Hardware.motorBl.setPower(blspeed * brake);
            Hardware.motorBr.setPower(brspeed * brake);
            Hardware.motorFr.setPower(frspeed * brake);

            //GAMEPAD 2

            //Slider
            if (gamepad2.dpad_up) {
                Hardware.slider.setTargetPosition(constant.high);
                Hardware.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Hardware.slider.setPower(constant.raiseSlider);
            }
            if (gamepad2.dpad_left) {
                Hardware.slider.setTargetPosition(constant.mid);
                Hardware.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (Hardware.slider.getCurrentPosition() >= Hardware.slider.getTargetPosition())
                    Hardware.slider.setPower(constant.lowerSlider);
                else if (Hardware.slider.getCurrentPosition() <= Hardware.slider.getTargetPosition())
                    Hardware.slider.setPower(constant.raiseSlider);
            }
            if (gamepad2.dpad_right){
                Hardware.slider.setTargetPosition(constant.low);
                Hardware.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (Hardware.slider.getCurrentPosition() >= Hardware.slider.getTargetPosition())
                    Hardware.slider.setPower(constant.lowerSlider);
                else if (Hardware.slider.getCurrentPosition() <= Hardware.slider.getTargetPosition())
                    Hardware.slider.setPower(constant.raiseSlider);
            }
            if (gamepad2.dpad_down) {
                Hardware.slider.setPower(0.0);
                Hardware.slider.setPower(constant.lowerSlider);
            }
            if (gamepad2.left_bumper){
                Hardware.slider.setTargetPosition(constant.collectIn);
                Hardware.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (Hardware.slider.getCurrentPosition() >= Hardware.slider.getTargetPosition())
                    Hardware.slider.setPower(constant.lowerSlider);
                else if (Hardware.slider.getCurrentPosition() <= Hardware.slider.getTargetPosition())
                    Hardware.slider.setPower(constant.raiseSlider);
            }

            if (Hardware.slider.getPower() < 0.0 && Hardware.limitSwitch.getState()) {
                Hardware.slider.setPower(-constant.stopSlider);
                Hardware.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Hardware.slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if ((Math.abs(Hardware.slider.getCurrentPosition() - Hardware.slider.getTargetPosition()) <= 10) && Hardware.slider.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                Hardware.slider.setPower(constant.stopSlider);
                Hardware.slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (-gamepad2.left_stick_y > 0)
                Hardware.slider.setPower(0.9);
            else if (-gamepad2.left_stick_y < 0)
                Hardware.slider.setPower(-0.9);
            else Hardware.slider.setPower(0.0);

            //claw

            if (gamepad2.a && Hardware.claw.getPosition() == constant.closedClaw){
                Hardware.claw.setPosition(constant.openClaw);
                sleep(200);
            }
            else if (gamepad2.a && Hardware.claw.getPosition() == constant.openClaw){
                Hardware.claw.setPosition(constant.closedClaw);
                sleep(200);
            }

            //arm

            if (Hardware.inside.getDistance(DistanceUnit.CM) < 3 && gamepad2.b && Hardware.inOut.getPosition()==constant.in){
                Hardware.inOut.setPosition(constant.out);
                sleep(200);
                Hardware.rotate.setPosition(constant.down);
            }
            else if (Hardware.inside.getDistance(DistanceUnit.CM) > 3 && gamepad2.b && Hardware.inOut.getPosition()==constant.in){
                Hardware.inOut.setPosition(constant.out);
                sleep(200);
                Hardware.rotate.setPosition(constant.up);
                }
            else if (Hardware.inOut.getPosition() == constant.out && gamepad2.b){
                Hardware.inOut.setPosition(constant.in);
                sleep(200);
            }

            if (Hardware.rotate.getPosition() == constant.up && gamepad2.x){
                Hardware.rotate.setPosition(constant.down);
                sleep(200);
            }
            else if (Hardware.rotate.getPosition() == constant.up && gamepad2.x){
                Hardware.rotate.setPosition(constant.down);
                sleep(200);
            }

            //intake

            if (gamepad2.right_trigger > 0.0)
                Hardware.intake.setPower(constant.intakeOn);
            else if (gamepad2.left_trigger > 0.0)
                Hardware.intake.setPower(-constant.intakeOn);
            else Hardware.intake.setPower(constant.intakeOff);

            if (Hardware.outside.getDistance(DistanceUnit.CM) < 9)
                Hardware.ledBand.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
            else Hardware.ledBand.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);


            telemetry.addData("Lift:", Hardware.slider.getCurrentPosition());
            telemetry.addData("Voltage:", Hardware.expansionHub.getInputVoltage(VoltageUnit.VOLTS));
            telemetry.addData("Switch:", Hardware.limitSwitch.getState());
            telemetry.update();
        }
    }


    public void normalize(double sp1, double sp2, double sp3, double sp4){
        if (opModeIsActive()){
            double max = Math.max(Math.max(Math.abs(sp1), Math.abs(sp2)), Math.max(Math.abs(sp3),Math.abs(sp4)));
            if (max > 1){
                sp1/=max;
                sp2/=max;
                sp3/=max;
                sp4/=max;
            }
        }
    }
}
