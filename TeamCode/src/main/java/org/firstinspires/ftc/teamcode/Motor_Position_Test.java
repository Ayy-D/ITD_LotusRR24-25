package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


@TeleOp(name="Motor_Position_Telemetry")

public class Motor_Position_Test extends LinearOpMode{
    String placeholder = "----";


    //Control Hub Servos
    private CRServo inR = null;
    private CRServo inL = null;
    private Servo inTwi = null;
    private Servo inUD = null; // SPM
    private Servo inArmR = null; // SPM
    private Servo inArmL = null; // SPM


    //Expansion Hub Motors
    private DcMotorEx sL = null;
    private DcMotorEx sR = null;

    //Expansion Hub Servos
    private Servo scR = null; //SPM
    private Servo scL = null; //SPM
    private Servo scUD = null; //SPM
    private Servo scC = null;

    //Sensors & Cameras
    private ColorSensor color = null;
    int red;
    int blue;
    int green;

    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        //LS Motors
        sL = hardwareMap.get(DcMotorEx.class, "slideL");
        sL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        sR = hardwareMap.get(DcMotorEx.class, "slideR");
        sR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        sR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sR.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        telemetry.addData(">>", "Press start to continue");
        telemetry.update();
        if (isStopRequested()) return;


        while (opModeIsActive())
        {

            telemetry.addData("-----", placeholder);

            telemetry.addData("sR Position", sR.getCurrentPosition());
            telemetry.addData("sR Current Draw", sR.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("sL Position", sL.getCurrentPosition());
            telemetry.addData("sL Current Draw", sL.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("-----", placeholder);
            telemetry.update();

        }
    }

}