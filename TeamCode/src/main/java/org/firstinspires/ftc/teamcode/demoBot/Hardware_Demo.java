package org.firstinspires.ftc.teamcode.demoBot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constant;

public class Hardware_Demo {


    public static DcMotorEx motorFl;
    public static DcMotorEx motorBl;
    public static DcMotorEx motorBr;
    public static DcMotorEx motorFr;
    public static DcMotorEx slider;

    public static Servo claw;

    //public static Servo angle;

    //public static LynxModule expansionHub;

    public static DigitalChannel limitSwitch;
    HardwareMap hwMap = null;


    public Hardware_Demo() {
    }

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        motorFl = hwMap.get(DcMotorEx.class, "motorFl");
        motorBl = hwMap.get(DcMotorEx.class, "motorBl");
        motorBr = hwMap.get(DcMotorEx.class, "motorBr");
        motorFr = hwMap.get(DcMotorEx.class, "motorFr");
        slider = hwMap.get(DcMotorEx.class, "slider");

        motorFl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFl.setPower(0.0);
        motorBl.setPower(0.0);
        motorBr.setPower(0.0);
        motorFr.setPower(0.0);
        slider.setPower(0.0);

        motorFl.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBl.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBr.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFr.setDirection(DcMotorSimple.Direction.FORWARD);
        slider.setDirection(DcMotorSimple.Direction.FORWARD);

        claw = hwMap.get(Servo.class, "claw");
        //angle = hwMap.get(Servo.class, "angle");

        //claw.setPosition(constant.closedClaw);
        //angle.setPosition(constant.collectAngle);

        limitSwitch = hwMap.get(DigitalChannel.class, "limitSwitch");

    }
}
