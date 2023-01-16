package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Constant;

public class Hardware {


    public static DcMotorEx motorFl;
    public static DcMotorEx motorBl;
    public static DcMotorEx motorBr;
    public static DcMotorEx motorFr;
    public static DcMotorEx intakeArm;
    public static DcMotorEx slider1;
    public static DcMotorEx slider2;
    public static DcMotorEx leftEncoder;
    public static DcMotorEx rightEncoder;
    public static DcMotorEx frontEncoder;

    public static Servo intakeClaw;
    public static Servo moveIntake;
    public static Servo mainClaw;
    public static Servo inOutAngle;
    public static Servo intake;


    public static RevColorSensorV3 intakeSensor;
    public static RevColorSensorV3 clawSensor;

    public static LynxModule expansionHub;

    public static DigitalChannel limitSwitch1;
    public static DigitalChannel limitSwitch2;
    public static DigitalChannel limitSwitchArm;

    public static AnalogInput potentiometer;
    HardwareMap hwMap =  null;

    public Hardware(){
    }

    public void init(HardwareMap ahwMap){

        hwMap = ahwMap;

        motorFl = hwMap.get(DcMotorEx.class, "frontLeft");
        motorBl = hwMap.get(DcMotorEx.class, "backLeft");
        motorBr = hwMap.get(DcMotorEx.class, "backRight");
        motorFr = hwMap.get(DcMotorEx.class, "frontRight");
        intakeArm = hwMap.get(DcMotorEx.class, "intakeArm");

        leftEncoder = hwMap.get(DcMotorEx.class, "backRight");
        rightEncoder = hwMap.get(DcMotorEx.class, "frontRight");
        frontEncoder = hwMap.get(DcMotorEx.class, "frontLeft");
        slider1 = hwMap.get(DcMotorEx.class, "sliderL");
        slider2 = hwMap.get(DcMotorEx.class, "sliderR");


        slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFl.setPower(0.0);
        motorBl.setPower(0.0);
        motorBr.setPower(0.0);
        motorFr.setPower(0.0);
        slider1.setPower(0.0);
        slider2.setPower(0.0);
        intakeArm.setPower(0.0);

        motorFl.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBl.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBr.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFr.setDirection(DcMotorSimple.Direction.FORWARD);
        slider1.setDirection(DcMotorSimple.Direction.REVERSE);
        slider2.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeArm.setDirection(DcMotorSimple.Direction.FORWARD);

        mainClaw   = hwMap.get(Servo.class, "mainClaw");
        inOutAngle = hwMap.get(Servo.class, "inOutAngle");
        intakeClaw = hwMap.get(Servo.class, "intakeClaw");
        moveIntake = hwMap.get(Servo.class, "moveIntake");
        intake     = hwMap.get(Servo.class, "intake");

        mainClaw  .setPosition(Constant.closedClaw);
        inOutAngle.setPosition(Constant.in);
        intakeClaw.setPosition(Constant.openIntake);
        moveIntake.setPosition(Constant.intakeCollect);
        intake    .setPosition(Constant.intakeStart);

        intakeSensor = hwMap.get(RevColorSensorV3.class, "intakeSensor");
        clawSensor    = hwMap.get(RevColorSensorV3.class, "clawSensor");

        limitSwitch1 = hwMap.get(DigitalChannel.class, "limitSwitch1");
        limitSwitch2 = hwMap.get(DigitalChannel.class, "limitSwitch2");
        limitSwitchArm = hwMap.get(DigitalChannel.class, "limitSwitchArm");

        potentiometer = hwMap.get(AnalogInput.class, "potentiometer");
    }
}
