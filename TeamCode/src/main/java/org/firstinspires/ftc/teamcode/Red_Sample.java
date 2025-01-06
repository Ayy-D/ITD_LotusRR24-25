package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "Red Sample", group = "Autonomous")
public class Red_Sample extends LinearOpMode {
    ElapsedTime time = new ElapsedTime();


    //Intake components


    //Linear Slide components
    public class Lift {
        private DcMotorEx sL;
        private DcMotorEx sR;

        public Lift(HardwareMap hardwareMap) {

            sL = hardwareMap.get(DcMotorEx.class, "slideL");
            sR = hardwareMap.get(DcMotorEx.class, "slideR");

            //RUN Encoders
            sL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //STOP reset
            sL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //BRAKE Behavior
            sR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            sL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //Set Direction
            sL.setDirection(DcMotorSimple.Direction.FORWARD);
            sR.setDirection(DcMotorSimple.Direction.REVERSE);

            sL.setTargetPosition(0);
            sR.setTargetPosition(0);

        }

        public class LiftAlignHighRung implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                sL.setPower(0.8);
                sR.setPower(0.8);
                sL.setTargetPosition(500);
                sR.setTargetPosition(500);
                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                return false;
            }
        }
        public Action liftAlignHighRung() {
            return new LiftAlignHighRung();
        }
        public class LiftScoreSpecimenHighRung implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sL.setPower(0.8);
                sR.setPower(0.8);
                sL.setTargetPosition(845);
                sR.setTargetPosition(845);
                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                return false;
            }
        }
        public Action liftScoreSpecimenHighRung() {
            return new LiftScoreSpecimenHighRung();
        }
    }

    public class LiftRotate {
        private DcMotorEx rotR;
        private DcMotorEx rotL;

        public LiftRotate(HardwareMap hardwareMap) {
            rotR = hardwareMap.get(DcMotorEx.class, "rotateR");
            rotL = hardwareMap.get(DcMotorEx.class, "rotateL");

            //RUN Encoders
            rotR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //BRAKE Behavior
            rotR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rotL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //Set Direction
            rotR.setDirection(DcMotorSimple.Direction.REVERSE);
            rotL.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftRotateSpecimenPickup implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rotR.setPower(0.4);
                    rotL.setPower(0.4);
                    initialized = true;
                }

                double posL = rotR.getCurrentPosition();
                double posR = rotL.getCurrentPosition();
                packet.put("Lift Rot L", posL);
                packet.put("lift Rot R", posR);

                // Check the motor
                if (posL < 205 || posR < 205) {
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action liftRotateSpecimenPickup() {
            return new LiftRotateSpecimenPickup();
        }

        public class LiftRotateSpecimenScore implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rotR.setPower(-0.4);
                    rotL.setPower(-0.4);
                    initialized = true;
                }

                double posL = rotR.getCurrentPosition();
                double posR = rotL.getCurrentPosition();
                packet.put("Lift L", posL);
                packet.put("lift R", posR);

                // Check the left motor
                if (posL > 2 || posR > 2) {
                    return true;
                } else {
                    rotL.setPower(0);
                    return false;
                }
            }
        }


        public Action liftRotateSpecimenScore() {
            return new LiftRotateSpecimenScore();
        }
    }

    public class Claw {
        private Servo scC;
        public Claw(HardwareMap hardwareMap) {
            scC = hardwareMap.get(Servo.class, "scClaw"); //0.27 close | 0.8 open
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                scC.setPosition(0.24);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                scC.setPosition(0.8);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }

    }

    public class Scoring {
        private Servo scL;
        private Servo scR;
        private Servo scUD;
        private Servo scC;
        private DcMotorEx sL;
        private DcMotorEx sR;
        private DcMotorEx rotR;
        private DcMotorEx rotL;
        public Scoring(HardwareMap hardwareMap) {
            scL = hardwareMap.get(Servo.class, "scArmL"); //0.95 goes toward intake, 0 goes outward from robot
            scR = hardwareMap.get(Servo.class, "scArmR"); //0.95 goes toward intake, 0 goes outward from robot
            scUD = hardwareMap.get(Servo.class, "scUD"); //1 is the position for depositing an element, 0.8 for intake, <0.8 to keep it up
            scUD.setDirection(Servo.Direction.REVERSE);
            scC = hardwareMap.get(Servo.class, "scClaw"); //0.27 close | 0.8 open

            sL = hardwareMap.get(DcMotorEx.class, "slideL");
            sR = hardwareMap.get(DcMotorEx.class, "slideR");

            rotR = hardwareMap.get(DcMotorEx.class, "rotateR");
            rotL = hardwareMap.get(DcMotorEx.class, "rotateL");

            //RUN Encoders
            rotR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //BRAKE Behavior
            rotR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rotL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //Set Direction
            rotR.setDirection(DcMotorSimple.Direction.REVERSE);
            rotL.setDirection(DcMotorSimple.Direction.FORWARD);

            //RUN Encoders

            sL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //BRAKE Behavior
            sR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            sL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //Set Direction
            sL.setDirection(DcMotorSimple.Direction.FORWARD);
            sR.setDirection(DcMotorSimple.Direction.REVERSE);


        }

        public class ScoringArmIntake implements Action {
            @Override
            public boolean run (@NonNull TelemetryPacket packet) {

                scL.setPosition(0.12); //verify
                scR.setPosition(0.12);
                scUD.setPosition(1); //verify
                scC.setPosition(0.8);

                sL.setPower(0.2);
                sR.setPower(0.2);
                sL.setTargetPosition(15);
                sR.setTargetPosition(15);
                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rotL.setPower(0.4);
                rotR.setPower(0.4);
                rotL.setTargetPosition(208);
                rotR.setTargetPosition(208);
                rotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                return false;
            }
        }
        public Action scoringArmIntake() {
            return new ScoringArmIntake();
        }

        public class ScoringArmScore implements Action {
            @Override
            public boolean run (@NonNull TelemetryPacket packet) {

                scL.setPosition(0.6); //verify
                scR.setPosition(0.6);
                scUD.setPosition(1); //verify
                scC.setPosition(0.25);

                sL.setPower(0.8);
                sR.setPower(0.8);
                sL.setTargetPosition(845);
                sR.setTargetPosition(845);
                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (sL.isBusy() && sR.isBusy()) {
                    telemetry.addData("Slide L", sL.getCurrentPosition());
                    telemetry.addData("Slide R", sR.getCurrentPosition());
                }

                rotL.setPower(0.4);
                rotR.setPower(0.4);
                rotL.setTargetPosition(50);
                rotR.setTargetPosition(50);
                rotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                return false;
            }
        }

        public Action scoringArmScore() {
            return new ScoringArmScore();
        }
    }


    public class IntakeHold {
        private CRServo inR;
        private CRServo inL;
        private Servo inUD;
        private Servo inArmR;
        private Servo inArmL;
        private Servo inTwist;

        public IntakeHold(HardwareMap hardwareMap) {
            inR = hardwareMap.get(CRServo.class, "inRight");
            inL = hardwareMap.get(CRServo.class, "inLeft");
            inUD = hardwareMap.get(Servo.class, "inUD");
            inArmL = hardwareMap.get(Servo.class, "inArmL");
            inArmR = hardwareMap.get(Servo.class, "inArmR");
            inTwist = hardwareMap.get(Servo.class, "inTwist");

        }

        public class IntakeHoldBase implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inR.setPower(0);
                inL.setPower(0);
                inUD.setPosition(0.3);
                inArmL.setPosition(0.14);
                inArmR.setPosition(0.14);
                inTwist.setPosition(0.3);
                return false;
            }
        }
        public Action intakeHoldBase() {
            return new IntakeHoldBase();
        }
    }


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-38, -60, Math.toRadians(270));
        Pose2d scoringPose = new Pose2d(-54,-54, Math.toRadians(225));
        Pose2d pickup1 = new Pose2d(-48,-48, Math.toRadians(270));
        Pose2d pickup2 = new Pose2d(-57,-48, Math.toRadians(270));
        Pose2d pickup3 = new Pose2d(-57,-46, Math.toRadians(305));
        //Pose2d intakePose = new Pose2d(-48, 46, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Claw claw = new Claw(hardwareMap);
        IntakeHold intakeHold = new IntakeHold(hardwareMap);
        Scoring scoring = new Scoring(hardwareMap);
        LiftRotate liftRotate = new LiftRotate(hardwareMap);
        Lift lift = new Lift(hardwareMap);



        //init to score
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(225))
                .waitSeconds(0.5);

        // Build the trajectory
        //score to pickup 1
        TrajectoryActionBuilder tab2 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(-47.5, -48), Math.toRadians(270))
                .waitSeconds(0.5);

        // pickup 1 to score
        TrajectoryActionBuilder tab3 = drive.actionBuilder(pickup1)
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(225))
                .waitSeconds(0.5);

        // score to pickup 2
        TrajectoryActionBuilder tab4 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(-57.5, -48), Math.toRadians(270))
                .waitSeconds(0.5);

        //pickup 2 to score
        TrajectoryActionBuilder tab5 = drive.actionBuilder(pickup2)
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(225))
                .waitSeconds(0.5);

        //score to pickup 3
        TrajectoryActionBuilder tab6 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(-57, -46), Math.toRadians(305))
                .waitSeconds(0.5);

        //pickup 3 to score
        TrajectoryActionBuilder tab7 = drive.actionBuilder(pickup3)
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(225))
                .waitSeconds(0.5);

        //park in observation zone
        Action TrajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .turnTo(Math.toRadians(270))
                .strafeTo(new Vector2d(34.5, -57))
                .build();



        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(intakeHold.intakeHoldBase());
        /***
         while (!isStopRequested() && !opModeIsActive()) {
         telemetry.update();
         }
         ***/
        // Wait for the start signal
        waitForStart();
        if (isStopRequested()) return;


        Action initToScoreTrajectory;
        Action scoreToPickup1;
        Action pickup1ToScore;
        Action scoreToPickup2;
        Action pickup2ToScore;
        Action scoreToPickup3;
        Action pickup3ToScore;

        initToScoreTrajectory = tab1.build();
        scoreToPickup1 = tab2.build();
        pickup1ToScore = tab3.build();
        scoreToPickup2 = tab4.build();
        pickup2ToScore = tab5.build();
        scoreToPickup3 = tab6.build();
        pickup3ToScore = tab7.build();

        Actions.runBlocking(
                new SequentialAction(
                        initToScoreTrajectory,
                        //slide score with bucket
                        scoreToPickup1,
                        //pickup sample
                        pickup1ToScore,
                        //slide score with bucket
                        scoreToPickup2,
                        //pickup sample
                        pickup2ToScore,
                        //slide score with bucket
                        scoreToPickup3,
                        //pickup sample
                        pickup3ToScore,
                        //slide score with bucket
                        TrajectoryActionCloseOut



                )
        );


    }
}
