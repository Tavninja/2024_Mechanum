
package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
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

  private final TalonFX m_LeftShooter = new TalonFX(5);
  private final TalonFX m_RightShooter = new TalonFX(7);
  private final TalonFX m_NoteFlipper = new TalonFX(0);
  private final TalonFX m_NotePusher = new TalonFX(6);
  private final TalonFX m_LeftClimer = new TalonFX(8);
  private final TalonFX m_RightClimer = new TalonFX(9);

  DigitalInput NoteOut = new DigitalInput(0);
  DigitalInput NoteIn = new DigitalInput(1);

  private final double shooterSpeed = .5;
  private final double NoteFlipperSpeed = .2;
  private final double NotePusherSpeed = .2;
  private final double LeftClimerSpeed = .2;
  private final double RightClimerSpeed = .2;

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

  // controllers ------------------------------------

  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_opperatorController = new XboxController(1);

  @Override
  public void teleopPeriodic() {

// Mecanum Drive Math --------------------------------------------------------------

    double y = m_driverController.getLeftY();
    double x = m_driverController.getLeftX();
    double x2 = m_driverController.getRightX();

    // Deadbands -------------------------
    if ((y < .1) && (y > -.1)) {
      y = 0;
    }
    if ((x < .1) && (x > -.1)) {
      x = 0;
    }
    if ((x2 < .1) && (x2 > -.1)) {
      x2 = 0;
    }

    double m_topLeftPower2 = (y + x + x2);
    double m_topRightPower2 = (y - x + x2);
    double m_bottomLeftPower2 = (y - x - x2);
    double m_bottomRightPower2 = (y + x - x2);

    //m_topLeftMotor.set(m_topLeftPower2);
    m_topRightMotor.set(m_topRightPower2);
    m_bottomLeftMotor.set(m_bottomLeftPower2);
    m_bottomRightMotor.set(m_bottomRightPower2);

// opperator controls ---------------------------------------------------------------

  //Note Shoot -------------------------------------
   if  (m_opperatorController.getRightTriggerAxis() > .5){ // shooter in
    m_LeftShooter.set(shooterSpeed);
    m_RightShooter.set(shooterSpeed);
        LeftShooterin = true;
        RightShooterin = true;
  } else if (m_opperatorController.getLeftTriggerAxis() > .5) { // shooter out
    m_LeftShooter.set(-shooterSpeed);
    m_RightShooter.set(-shooterSpeed);
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

  // note out and flip ------------------------------
  if (m_opperatorController.getRightBumperPressed()){ //note out
    m_NotePusher.set(NotePusherSpeed);
        NotePusherout = true;
  } else if (NoteOut.get() && m_opperatorController.getRightBumperPressed()){ //note hit back and flip
    m_NotePusher.set(0);
    m_NoteFlipper.set(NoteFlipperSpeed);
        NoteFlipperout = true;
        NotePusherout = false;
  }else{ // note pusher stop
    m_NotePusher.set(0);
    m_NoteFlipper.set(0);
        NotePusherout = false;
        NoteFlipperout = false;
  }

  // note in and stop -------------------------------
  if (m_opperatorController.getLeftBumperPressed()){ //note in
    m_NotePusher.set(-NotePusherSpeed);
        NotePusherin = true;
  } else if (NoteIn.get() && m_opperatorController.getLeftBumperPressed()){ // note hit bot
    m_NotePusher.set(0);
        NotePusherin = true;
  } else { // note pusher stop
    m_NotePusher.set(0);
    m_NoteFlipper.set(0);
        NotePusherin = false;
  }

  // reset flipper ----------------------------------
  if (m_opperatorController.getAButton()) { // flipper left
    m_NoteFlipper.set(NoteFlipperSpeed);
    NoteFlipperin = true;
  } else if (m_opperatorController.getBButton()){ // flipper right
    m_NoteFlipper.set(-NoteFlipperSpeed);
    NoteFlipperout = true;
  } else { // flipper stop
    m_NoteFlipper.set(0);
  }


  //Climer ------------------------------------------

  if (m_driverController.getLeftBumperPressed()){
    m_LeftClimer.set(LeftClimerSpeed);
        LeftClimerin = true;
  } else if (m_driverController.getLeftTriggerAxis() > .5){
    m_LeftClimer.set(-LeftClimerSpeed);
      LeftClimerout = true;
  } else if (m_driverController.getRightBumperPressed()){
    m_RightClimer.set(RightClimerSpeed);
        RightClimerin = true;
  } else if (m_driverController.getRightTriggerAxis() > .5){
    m_RightClimer.set(-RightClimerSpeed);
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

  SmartDashboard.putBoolean("left_Climerin", LeftClimerin.booleanValue());
  SmartDashboard.putBoolean("left_Climerout", LeftClimerout.booleanValue());

  SmartDashboard.putBoolean("Right_Climerin", RightClimerin.booleanValue());
  SmartDashboard.putBoolean("Right_Climerout", RightClimerout.booleanValue());

  SmartDashboard.putBoolean("left_Shooterin", LeftShooterin.booleanValue());
  SmartDashboard.putBoolean("left_Shooterout", LeftShooterout.booleanValue());

  SmartDashboard.putBoolean("Right_Shooterin", RightShooterin.booleanValue());
  SmartDashboard.putBoolean("Right_Shooterout", RightShooterout.booleanValue());

  SmartDashboard.putBoolean("Note_Pusherin", NotePusherin.booleanValue());
  SmartDashboard.putBoolean("Note_Pusherout", NotePusherout.booleanValue());

  SmartDashboard.putBoolean("Note_Fliperin", NoteFlipperin.booleanValue());
  SmartDashboard.putBoolean("Note_Fliperout", NoteFlipperout.booleanValue());
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
// Auto Switcher -------------------------------------------
 @Override
 public void robotInit() {

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

 // Autonomous ---------------------------------------------

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
