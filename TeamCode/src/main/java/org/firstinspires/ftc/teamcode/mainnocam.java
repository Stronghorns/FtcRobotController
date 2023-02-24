package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;

@TeleOp(name = "mainnocam")
public class mainnocam extends LinearOpMode {

  private DcMotor linear_slide = null;
  private DcMotor right_front = null;
  private DcMotor right_back = null;
  private DcMotor left_back = null;
  private DcMotor left_front = null;
  private Servo hook_servo = null;
  private RevBlinkinLedDriver LED_RevBlinkinLedDriver = null;
  private BNO055IMU imu = null;
  private VuforiaCurrentGame vuforiaPOWERPLAY = null;

  int maxVelocity;
  double speed;
  int maxHeight;
  boolean slow;
  boolean closed;
  int linearSlide;
  float BotRotation;
  float SetRotation;
  double averageSpeed;
  float forward;
  float rotate;
  float strafe;

  /**
   * Describe this function...
   */
  private void Startup() {
    maxVelocity = 2500;
    maxHeight = 4300;
    speed = 0.6;
    closed = true;
    slow = false;
    SetRotation = 0;
    linearSlide = linear_slide.getCurrentPosition();
    right_front.setDirection(DcMotorSimple.Direction.REVERSE);
    right_back.setDirection(DcMotorSimple.Direction.REVERSE);
    left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    hook_servo.setPosition(0);
  }

  /**
   * Describe this function...
   */
  private void Button() {
    boolean A2 = false;
    boolean Left2 = false;
    boolean Right = false;

    if (A2 == false && gamepad1.a == true) {
      A();
    }
    A2 = gamepad1.a;
    if (Left2 == false && gamepad1.dpad_left == true) {
      Left();
    }
    Left2 = gamepad1.dpad_left;
    if (Right == false && gamepad1.dpad_left == true) {
      Left();
    }
    Right = gamepad1.dpad_right;
  }

  /**
   * Describe this function...
   */
  private void LinearSlide() {
    double liftpos = 0;

    if (gamepad1.right_bumper) {
      if (linear_slide.getCurrentPosition() > maxHeight) {
        linear_slide.setPower(0);
        linearSlide = linear_slide.getCurrentPosition();
      } else {
        linear_slide.setPower(0.75);
        linearSlide = linear_slide.getCurrentPosition();
      }
    } else if (gamepad1.left_bumper) {
      linear_slide.setPower(-0.5);
      linearSlide = linear_slide.getCurrentPosition();
    } else {
      if (linear_slide.getCurrentPosition() != linearSlide) {
        linear_slide.setPower(-Double.parseDouble(JavaUtil.formatNumber(1 - linear_slide.getCurrentPosition() / liftpos, 4)));
      }
    }
  }

  /**
   * Describe this function...
   */
  private void Y() {
    speed = 0.3;
    slow = true;
  }

  /**
   * Describe this function...
   */
  private void X() {
    hook_servo.setPosition(0);
    closed = true;
  }

  /**
   * Describe this function...
   */
  private void Telemetry() {
    if (linear_slide.getCurrentPosition() > 1500 && linear_slide.getCurrentPosition() < 2500) {
      telemetry.addData("Height", "1st Bar");
    } else if (linear_slide.getCurrentPosition() > 2500 && linear_slide.getCurrentPosition() < 3530) {
      telemetry.addData("Height", "2nd Bar");
    } else if (linear_slide.getCurrentPosition() > 3530) {
      telemetry.addData("Height", "3rd Bar");
    } else {
      telemetry.addData("Height", "Pickup Cone");
    }
    telemetry.addData("Angle", Math.round(BotRotation) + " / " + Math.round(SetRotation) + " (" + Math.round(BotRotation - SetRotation) + ")");
    telemetry.addData("Linear Slide", linear_slide.getCurrentPosition() + " / " + linearSlide);
    telemetry.addData("Average Speed", averageSpeed);
    telemetry.addData("Claw Closed", closed);
    telemetry.addData("Slow Mode", slow);
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void B() {
    hook_servo.setPosition(0.25);
    closed = false;
  }

  /**
   * Describe this function...
   */
  private void Left() {
    if (slow == true) {
      speed = 0.6;
      slow = false;
    } else {
      speed = 0.3;
      slow = true;
    }
  }

  /**
   * Describe this function...
   */
  private void A() {
    if (closed == true) {
      B();
    } else {
      X();
    }
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    linear_slide = hardwareMap.get(DcMotor.class, "linear_slide");
    right_front = hardwareMap.get(DcMotor.class, "right_front");
    right_back = hardwareMap.get(DcMotor.class, "right_back");
    left_back = hardwareMap.get(DcMotor.class, "left_back");
    left_front = hardwareMap.get(DcMotor.class, "left_front");
    hook_servo = hardwareMap.get(Servo.class, "hook_servo");
    LED_RevBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    vuforiaPOWERPLAY = new VuforiaCurrentGame();

    LED_RevBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
    // Put initialization blocks here.
    imu.initialize(new BNO055IMU.Parameters());
    waitForStart();
    if (opModeIsActive()) {
      Startup();
      while (opModeIsActive()) {
        BotRotation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        forward = gamepad1.left_stick_y;
        rotate = gamepad1.right_stick_x;
        strafe = -(gamepad1.left_stick_x / 2);
        averageSpeed = JavaUtil.sumOfList(JavaUtil.createListWith(forward, rotate, strafe)) / 3;
        if (rotate != 0) {
          SetRotation = BotRotation;
        }
        LinearSlide();
        Velocity2();
        Button();
        Telemetry();
      }
    }
    vuforiaPOWERPLAY.deactivate();

    vuforiaPOWERPLAY.close();
  }

  /**
   * Describe this function...
   */
  private void Velocity2() {
    ((DcMotorEx) left_front).setVelocity((((forward - (rotate + strafe)) - 0.2 * gamepad1.left_stick_y) * speed - 0) * maxVelocity);
    ((DcMotorEx) left_back).setVelocity((((forward - (rotate - strafe)) + -0.2 * gamepad1.left_stick_y) * speed - 0) * maxVelocity);
    ((DcMotorEx) right_front).setVelocity((((forward + rotate + strafe) - 0.2 * gamepad1.left_stick_y) * speed + 0) * maxVelocity);
    ((DcMotorEx) right_back).setVelocity(((forward + (rotate - strafe) + -0.2 * gamepad1.left_stick_y) * speed + 0) * maxVelocity);
  }
}