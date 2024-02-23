
package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {

  //Set Motor values --------------------------------------------------------------
  private final static CANSparkMax m_bottomLeftMotor = new CANSparkMax(4, MotorType.kBrushed);
  private final static CANSparkMax m_bottomRightMotor = new CANSparkMax(3, MotorType.kBrushed);
  private final static CANSparkMax m_topLeftMotor = new CANSparkMax(1, MotorType.kBrushed);
  private final static CANSparkMax m_topRightMotor = new CANSparkMax(2, MotorType.kBrushed);
  private final static CANSparkMax m_NotePusher = new CANSparkMax(6, MotorType.kBrushless);

  private final TalonFX m_LeftShooter = new TalonFX(5);
  private final TalonFX m_RightShooter = new TalonFX(7);
  private final TalonFX m_NoteFlipper = new TalonFX(10);
  private final TalonFX m_LeftClimer = new TalonFX(8);
  private final TalonFX m_RightClimer = new TalonFX(9);

  DigitalInput NoteOut = new DigitalInput(0);
  DigitalInput NoteIn = new DigitalInput(1);

  private final double fastShooterSpeed = .35;
  private final double NoteFlipperSpeed = .05;
  private final double NotePusherSpeed = .1;
  private final double climerSpeed = .35;
  private final double slowShooterSpeed = .35;

  private SlewRateLimiter filter = new SlewRateLimiter(2.5);

  UsbCamera camera1;
  UsbCamera camera2;
  VideoSink server; 

  // booleans ---------------------------------------
 
  private static Boolean LeftShooterout = false;
  private static Boolean LeftShooterin = false;

  private static Boolean RightShooterout = false;
  private static Boolean RightShooterin = false;

  private static Boolean NoteFlipperout = false;
  private static Boolean NoteFlipperin = false;

  private static Boolean NotePusherout = false;
  private static Boolean NotePusherin = false;

  private static Boolean LeftClimerout = false;
  private static Boolean LeftClimerin = false;

  private static Boolean RightClimerout = false;
  private static Boolean RightClimerin = false;

  private static boolean LimitSwitchin = false;
  private static boolean LimitSwitchout = false;

  private static boolean test = false;

  // controllers ------------------------------------

  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_opperatorController = new XboxController(1);

  @Override
  public void teleopPeriodic() {

// Mecanum Drive Math --------------------------------------------------------------

    //------------------------------------------------

    double y = m_driverController.getLeftY()*-1;
    double x = m_driverController.getLeftX();
    double x2 = m_driverController.getRightX();

    double kp = 0;
    double ki = 0;
    double kd = 0;

    PIDController pidController = new PIDController(kp,ki,kd);

    double Y_Speed = 0;
    double X_Speed = 0;
    double X2_Speed = 0;

    m_topLeftMotor.set(pidController.calculate(m_driverController.getLeftY(),Y_Speed));

    m_topLeftMotor.setInverted(true);
    m_topRightMotor.setInverted(false);
    m_bottomLeftMotor.setInverted(true);
    m_bottomRightMotor.setInverted(false);

    // Deadbands -------------------------
    if ((y < .2) && (y > -.2)) {
      y = 0;
    }
    if ((x < .2) && (x > -.2)) {
      x = 0;
    }
    if ((x2 < .2) && (x2 > -.2)) {
      x2 = 0;
    }

  // --------------------------------------------------------

    double m_topLeftPower = (y + x + x2);
    double m_topRightPower = (y - x - x2);
    double m_bottomLeftPower = (y - x + x2);
    double m_bottomRightPower =(y + x - x2);

    double m_topLeftPower2 = (Y_Speed + X_Speed + X2_Speed);
    double m_topRightPower2 = (Y_Speed - X_Speed - X2_Speed);
    double m_bottomLeftPower2 = (Y_Speed - X_Speed + X2_Speed);
    double m_bottomRightPower2 =(Y_Speed + X_Speed - X2_Speed);

    //m_topLeftMotor.set(m_topLeftPower);
    //m_topRightMotor.set(m_topRightPower);
    //m_bottomLeftMotor.set(m_bottomLeftPower);
    //m_bottomRightMotor.set(m_bottomRightPower);

// opperator controls ---------------------------------------------------------------


/*
Spark spark = new Spark(0);

// Limit switch on DIO 2
DigitalInput limit = new DigitalInput(2);

public void autonomousPeriodic() {
    // Runs the motor forwards at half speed, unless the limit is pressed
    if(!limit.get()) {
        spark.set(.5);
    } else {
        spark.set(0);
    }
} */
  //Note Shoot -------------------------------------
   if  (m_opperatorController.getRightTriggerAxis() > .5){ // shooter in
    m_LeftShooter.set(-fastShooterSpeed);
    m_RightShooter.set(fastShooterSpeed);
        LeftShooterin = true;
        RightShooterin = true;
  } else if (m_opperatorController.getLeftTriggerAxis() > .5) { // shooter out
    m_LeftShooter.set(fastShooterSpeed);
    m_RightShooter.set(-fastShooterSpeed);
        LeftShooterout = true;
        RightShooterout = true;
  } else if (m_opperatorController.getXButton()){ // slow shoot out
    m_LeftShooter.set(slowShooterSpeed);
    m_RightShooter.set(-slowShooterSpeed);
        LeftShooterout = true;
        RightShooterout = true;
  } else if (m_opperatorController.getYButton()){ // shoot slow in
    m_RightShooter.set(slowShooterSpeed);
    m_LeftShooter.set(-slowShooterSpeed);
        LeftShooterout = true;
        RightShooterout = true;
  } else { // shooter stop
    m_LeftShooter.set(0); 
    m_RightShooter.set(0);
        LeftShooterin = false;
        LeftShooterout = false;
        RightShooterin = false;
        RightShooterout = false;
  }
 
  /*// note out and flip ------------------------------
  if (m_opperatorController.getXButton()){ //note out
    m_NotePusher.set(NotePusherSpeed);
        NotePusherout = true;
  }  else if (NoteOut.get() && m_opperatorController.getRightBumper()){ //note hit back and flip
    m_NotePusher.set(0);
    m_NoteFlipper.set(NoteFlipperSpeed);
        NoteFlipperout = true;
        NotePusherout = false;
        LimitSwitchout = true;
  } else{ // note pusher stop
    m_NotePusher.set(0);
    m_NoteFlipper.set(0);
        NotePusherout = false;
        NoteFlipperout = false;
        LimitSwitchout = false;
  }


  //test limit Switch -------------------------------
if (NoteOut.get() == false){
 test = true;
} else {
  test = false;
}
*/
  // note in and stop -------------------------------
  if (m_opperatorController.getRightBumper()){ //note in
    m_NotePusher.set(-NotePusherSpeed);
        NotePusherin = true;
        NotePusherout = false;
  } else if (m_opperatorController.getLeftBumper()){
    m_NotePusher.set(NotePusherSpeed);
        NotePusherin = false;
        NotePusherout = true;
  } else if (m_opperatorController.getLeftBumper() && (NoteOut.get() == false)){
      m_NotePusher.set(0);
      LimitSwitchout = true;
  } else if (m_opperatorController.getRightBumper() && (NoteIn.get() == false)){
      m_NotePusher.set(0);
      LimitSwitchin = true;
  } else { // note pusher stop
    m_NotePusher.set(0);
        NotePusherin = false;
        NotePusherout = false;
        LimitSwitchin = false;
        LimitSwitchout = false;
  }

    SmartDashboard.putData(NoteOut);
    SmartDashboard.putData(NoteIn);

  // reset flipper ----------------------------------
  if (m_opperatorController.getAButton()) { // flipper left
    m_NoteFlipper.set(NoteFlipperSpeed);
    NoteFlipperin = true;
  } else if (m_opperatorController.getBButton()){ // flipper right
    m_NoteFlipper.set(-NoteFlipperSpeed);
    NoteFlipperout = true;
  } else { // flipper stop
    m_NoteFlipper.set(0);
    NoteFlipperout = false;
    NoteFlipperin = false;
  }


  //Climer ------------------------------------------

  if (m_driverController.getLeftBumper()){
    m_LeftClimer.set(climerSpeed);
        LeftClimerin = true;
  } else if (m_driverController.getLeftTriggerAxis() > .5){
    m_LeftClimer.set(-climerSpeed);
      LeftClimerout = true;
  } else if (m_driverController.getRightBumper()){
    m_RightClimer.set(-climerSpeed);
        RightClimerin = true;
  } else if (m_driverController.getRightTriggerAxis() > .5){
    m_RightClimer.set(climerSpeed);
        RightClimerout = true;
  } else {
    m_LeftClimer.set(0);
    m_RightClimer.set(0);
        LeftClimerin = false;
        RightClimerin = false;
        LeftClimerout = false;
        RightClimerout = false;
  }

// SmartDashboard ----------------------------------------------------

    SmartDashboard.putNumber("y", y);
    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("2x", x2);
  
 
  SmartDashboard.putBoolean("left_Climer_in", LeftClimerin.booleanValue());
  SmartDashboard.putBoolean("left_Climer_out", LeftClimerout.booleanValue());

  SmartDashboard.putBoolean("Right_Climer_in", RightClimerin.booleanValue());
  SmartDashboard.putBoolean("Right_Climer_out", RightClimerout.booleanValue());

  SmartDashboard.putBoolean("left_Shooter_in", LeftShooterin.booleanValue());
  SmartDashboard.putBoolean("left_Shooter_out", LeftShooterout.booleanValue());

  SmartDashboard.putBoolean("Right_Shooter_in", RightShooterin.booleanValue());
  SmartDashboard.putBoolean("Right_Shooter_out", RightShooterout.booleanValue());

  SmartDashboard.putBoolean("Note_Pusher_in", NotePusherin.booleanValue());
  SmartDashboard.putBoolean("Note_Pusher_out", NotePusherout.booleanValue());

  SmartDashboard.putBoolean("Note_Fliper_in", NoteFlipperin.booleanValue());
  SmartDashboard.putBoolean("Note_Fliper_out", NoteFlipperout.booleanValue());

  SmartDashboard.putBoolean("Limit_Switch_in", LimitSwitchin);
  SmartDashboard.putBoolean("Limit_Switch_out", LimitSwitchout);

  SmartDashboard.putNumber("F topLeftPower2", (m_topLeftPower2));
  SmartDashboard.putNumber("F toprightPower", (m_topRightPower2));
  SmartDashboard.putNumber("F bottomLeftPower2", (m_bottomLeftPower2));
  SmartDashboard.putNumber("F m_bottomRightPower2", (m_bottomRightPower2));

  SmartDashboard.putNumber("topLeftPower2", (m_topLeftPower));
  SmartDashboard.putNumber("toprightPower", (m_topRightPower));
  SmartDashboard.putNumber("bottomLeftPower2", (m_bottomLeftPower));
  SmartDashboard.putNumber("m_bottomRightPower2", (m_bottomRightPower));

  SmartDashboard.putNumber("F y", filter.calculate(y));
  SmartDashboard.putNumber("F x", filter.calculate(x));
  SmartDashboard.putNumber("F x2",filter.calculate(x2));
  
}
// Auto Time Contants ------------------------------------------------

private Timer timer;
 private double step1Time = 1;             //
 private double step2Time = step1Time + 1; //
 private double step3Time = step2Time + 1; //
 private double step4Time = step3Time + 1; //
 private double step5Time = step4Time + 1; //
 private double step6Time = step5Time + 1; //
 private static final String kDefaultAuto = "Default";
 private static final String kCustomAuto = "My Auto";
 private String m_autoSelected;
 private final SendableChooser<String> m_chooser = new SendableChooser<>();

// Movement functions ------------------------------------------------

 static void Forward(){
  m_topLeftMotor.set(.5);
  m_topRightMotor.set(.5);
  m_bottomLeftMotor.set(.5);
  m_bottomRightMotor.set(.5);
 }
 static void Backwards(){
  m_topLeftMotor.set(-.5);
  m_topRightMotor.set(-.5);
  m_bottomLeftMotor.set(-.5);
  m_bottomRightMotor.set(-.5);
 }
 static void Left(){
  m_topLeftMotor.set(.5);
  m_topRightMotor.set(-.5);
  m_bottomLeftMotor.set(-.5);
  m_bottomRightMotor.set(.5);
 }
 static void Right(){
  m_topLeftMotor.set(-.5);
  m_topRightMotor.set(.5);
  m_bottomLeftMotor.set(.5);
  m_bottomRightMotor.set(-.5);
 }
 static void Stoped(){
  m_topLeftMotor.set(0);
  m_topRightMotor.set(0);
  m_bottomLeftMotor.set(0);
  m_bottomRightMotor.set(0);
 }
@Override
 public void robotInit() {

// Camera ----------------------------------------------------------
    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);
    server = CameraServer.getServer();

    camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

// Auto Switcher ----------------------------------------------------
   m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
   m_chooser.addOption("My Auto", kCustomAuto);
   SmartDashboard.putData("Auto choices", m_chooser);
 }

 @Override
 public void autonomousInit() {
   m_autoSelected = m_chooser.getSelected();
   System.out.println("Auto selected: " + m_autoSelected);

   timer = new Timer();
   timer.start();
 }

 // Autonomous ---------------------------------------------------

 @Override
 public void autonomousPeriodic() {
   switch (m_autoSelected) {
     case kCustomAuto:
       Stoped();
       break;
     case kDefaultAuto:
     default:
       if (timer.get() <= step1Time) {
         Forward();
       } else if (timer.get() <= step2Time) {
         Backwards();
       } else if (timer.get() <= step3Time) {
         Left();
       }else if (timer.get() <= step4Time) {
         Right();
       } else if (timer.get() <= step5Time) {
         Stoped();
       } else if (timer.get() <= step6Time){
         Stoped();
       } else {
         Stoped();
       }
       break;
   }
  }

// End ----------------------------------------------------------

  @Override
  public void robotPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
