
                        package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

                @TeleOp(name = "MechanumDrive1 (Java)")
                public class BasicOpMode_Linear extends LinearOpMode {

                    private DcMotor leftFront;
                    private DcMotor leftBack;
                    private DcMotor Left_Lift;
                    private DcMotor rightFront;
                    private DcMotor rightBack;
                    private Servo Extend_Intake;
                    private Servo Elbow_Intake;
                    private CRServo Wheel_Intake;
                    private DcMotor RightLift;
                    private Servo Dump;

                    /**
                     * This function is executed when this Op Mode is selected from the Driver Station.
                     */
                    @Override
                    public void runOpMode() {
                        double ServoSpeed;
                        double ServoPosition;
                        double ElbowServoSpd;
                        double ElbowServoPos;
                        double DumpServoSpd;
                        double DumpServoPos;
                        float Vertical;
                        float Horizontal;
                        float Pivot;

                        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
                        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
                        Left_Lift = hardwareMap.get(DcMotor.class, "Left_Lift");
                        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
                        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
                        Extend_Intake = hardwareMap.get(Servo.class, "Extend_Intake");
                        Elbow_Intake = hardwareMap.get(Servo.class, "Elbow_Intake");
                        Wheel_Intake = hardwareMap.get(CRServo.class, "Wheel_Intake");
                        RightLift = hardwareMap.get(DcMotor.class, "Right Lift");
                        Dump = hardwareMap.get(Servo.class, "Dump");

                        leftFront.setDirection(DcMotor.Direction.REVERSE);
                        leftBack.setDirection(DcMotor.Direction.REVERSE);
                        Left_Lift.setDirection(DcMotor.Direction.REVERSE);
                        // Put initialization blocks here.
                        waitForStart();
                        ServoSpeed = 0.1;
                        ServoPosition = 0;
                        ElbowServoSpd = 0.1;
                        ElbowServoPos = 0.5;
                        DumpServoSpd = 0.1;
                        DumpServoPos = 0.5;
                        if (opModeIsActive()) {
                            while (opModeIsActive()) {
                                Vertical = gamepad1.left_stick_y;
                                Horizontal = -gamepad1.left_stick_x;
                                Pivot = -gamepad1.right_stick_x;
                                if (gamepad1.left_bumper) {
                                    rightFront.setPower(-Pivot + (Vertical - Horizontal));
                                    rightBack.setPower(-Pivot + Vertical + Horizontal);
                                    leftFront.setPower(Pivot + Vertical + Horizontal);
                                    leftBack.setPower(Pivot + (Vertical - Horizontal));
                                } else if (gamepad1.right_bumper) {
                                    rightFront.setPower((-Pivot + (Vertical - Horizontal)) * 0.25);
                                    rightBack.setPower((-Pivot + Vertical + Horizontal) * 0.25);
                                    leftFront.setPower((Pivot + Vertical + Horizontal) * 0.25);
                                    leftBack.setPower((Pivot + (Vertical - Horizontal)) * 0.25);
                                } else {
                                    rightFront.setPower((-Pivot + (Vertical - Horizontal)) * 0.5);
                                    rightBack.setPower((-Pivot + Vertical + Horizontal) * 0.5);
                                    leftFront.setPower((Pivot + Vertical + Horizontal) * 0.5);
                                    leftBack.setPower((Pivot + (Vertical - Horizontal)) * 0.5);
                                }
                                // use gamepad x and b for servospeed
                                if (gamepad2.b) {
                                    ServoPosition += ServoSpeed;
                                    ElbowServoPos += -ElbowServoSpd;
                                }
                                if (gamepad2.x) {
                                    ServoPosition += -ServoSpeed;
                                    ElbowServoPos += ElbowServoSpd;
                                }
                                if (gamepad2.y) {
                                    ElbowServoPos = 0.5;
                                }
                                ServoPosition = Math.min(Math.max(ServoPosition, 0.3), 0.8);
                                ElbowServoPos = Math.min(Math.max(ElbowServoPos, 0.1), 0.65);
                                Extend_Intake.setPosition(ServoPosition);
                                Elbow_Intake.setPosition(ElbowServoPos);
                                Wheel_Intake.setPower(gamepad2.left_stick_y);
                                if (gamepad2.right_bumper) {
                                    Left_Lift.setPower(-(gamepad2.right_stick_y * 0.25));
                                    RightLift.setPower(gamepad2.right_stick_y * 0.25);
                                } else if (gamepad2.left_bumper) {
                                    Left_Lift.setPower(-(gamepad2.right_stick_y * 0.5));
                                    RightLift.setPower(gamepad2.right_stick_y * 0.5);
                                } else {
                                    Left_Lift.setPower(-gamepad2.right_stick_y);
                                    RightLift.setPower(gamepad2.right_stick_y);
                                }
                                if (gamepad1.a) {
                                    DumpServoPos += ServoSpeed;
                                }
                                if (gamepad1.b) {
                                    DumpServoPos += -ServoSpeed;
                                }
                                DumpServoPos = Math.min(Math.max(DumpServoPos, 0.1), 0.9);
                                Dump.setPosition(DumpServoPos);
                                telemetry.addData("horizontal", Horizontal);
                                telemetry.addData("vertical", Vertical);
                                telemetry.addData("pivot", Pivot);
                                telemetry.addData("Elbow", ElbowServoPos);
                                telemetry.addData("Dump", DumpServoPos);
                                telemetry.update();
                                sleep(20);
                                telemetry.update();
                                telemetry.addData("Left Back Pow", leftBack.getPower());
                                telemetry.addData("Left Front Pow", leftFront.getPower());
                                telemetry.addData("Right Back Pow", rightBack.getPower());
                                telemetry.addData("Right Front Pow", rightFront.getPower());
                                telemetry.addData("Extender Power", ServoPosition);
                            }
                        }
                    }
                }


/*@TeleOp(name="FSM Example")
public class FSMExample extends OpMode {
    // An Enum is used to represent lift states.
    // (This is one thing enums are designed to do)
    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT
    };

    // The liftState variable is declared out here
    // so its value persists between loop() calls
    LiftState liftState = LiftState.LIFT_START;

    // Some hardware access boilerplate; these would be initialized in init()
    // the lift motor, it's in RUN_TO_POSITION mode
    public DcMotorEx liftMotor;

    // the dump servo
    public Servo liftDump;
    // used with the dump servo, this will get covered in a bit
    ElapsedTime liftTimer = new ElapsedTime();

    final double DUMP_IDLE; // the idle position for the dump servo
    final double DUMP_DEPOSIT; // the dumping position for the dump servo

    // the amount of time the dump servo takes to activate in seconds
    final double DUMP_TIME;

    final int LIFT_LOW; // the low encoder position for the lift
    final int LIFT_HIGH; // the high encoder position for the lift

    public void init() {
        liftTimer.reset();

        // hardware initialization code goes here
        // this needs to correspond with the configuration used
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftDump = hardwareMap.get(Servo.class, "liftDump");

        liftMotor.setTargetPosition(LIFT_LOW);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void loop() {
        liftMotor.setPower(1.0);

        switch (liftState) {
            case LIFT_START:
                // Waiting for some input
                if (gamepad1.x) {
                    // x is pressed, start extending
                    liftMotor.setTargetPosition(LIFT_HIGH);
                    liftState = LiftState.LIFT_EXTEND;
                }
                break;
            case LIFT_EXTEND:
                 // check if the lift has finished extending,
                 // otherwise do nothing.
                if (Math.abs(liftMotor.getCurrentPosition() - LIFT_HIGH) < 10) {
                    // our threshold is within
                    // 10 encoder ticks of our target.
                    // this is pretty arbitrary, and would have to be
                    // tweaked for each robot.

                    // set the lift dump to dump
                    liftDump.setTargetPosition(DUMP_DEPOSIT);

                    liftTimer.reset();
                    liftState = LiftState.LIFT_DUMP;
                }
                break;
            case LIFT_DUMP:
                if (liftTimer.seconds() >= DUMP_TIME) {
                    // The robot waited long enough, time to start
                    // retracting the lift
                    liftDump.setTargetPosition(DUMP_IDLE);
                    liftMotor.setTargetPosition(LIFT_LOW);
                    liftState = LiftState.LIFT_RETRACT;
                }
                break;
            case LIFT_RETRACT:
                if (Math.abs(liftMotor.getCurrentPosition() - LIFT_LOW) < 10) {
                    liftState = LiftState.LIFT_START;
                }
                break;
            default:
                 // should never be reached, as liftState should never be null
                 liftState = LiftState.LIFT_START;
        }

        // small optimization, instead of repeating ourselves in each
        // lift state case besides LIFT_START for the cancel action,
        // it's just handled here
        if (gamepad1.y && liftState != LiftState.LIFT_START) {
            liftState = LiftState.LIFT_START;
        }

        // mecanum drive code goes here
        // But since none of the stuff in the switch case stops
        // the robot, this will always run!
        updateDrive(gamepad1, gamepad2);
   }
}

 */