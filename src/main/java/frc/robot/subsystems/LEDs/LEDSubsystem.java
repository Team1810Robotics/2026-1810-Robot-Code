package frc.robot.subsystems.LEDs;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import javax.lang.model.util.ElementScanner14;

import org.w3c.dom.css.RGBColor;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private CANdle m_candle;

    public LEDSubsystem() { 
        m_candle = new CANdle(35);
        configureCANdle();
    }

    public static RGBWColor LEDColor = new RGBWColor(255, 0, 0, 0); //This Defaults to red and can be changed through a command

    private void configureCANdle() {
        CANdleConfiguration config = new CANdleConfiguration();

        config.LED.BrightnessScalar = .5;
        config.LED.StripType = StripTypeValue.GRB; 
        config.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;

        config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;

        m_candle.getConfigurator().apply(config);
    }

  public void setSolidColor(int[] color) {
    int r = color[0];
    int g = color[1];
    int b = color[2];

    RGBWColor rgbwColor = new RGBWColor(g, r, b);
    
    m_candle.setControl(new SolidColor(8, 999).withColor(rgbwColor));
    }   

     public void initLEDColor() {
        setSolidColor(new int[]{0,255,0});
    }

    // private static final RGBWColor kViolet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
    // private static final RGBWColor kRed = RGBWColor.fromHex("#D9000000").orElseThrow();

    //public static RGBWColor LEDColor = new RGBWColor(255, 0, 0, 0); //This Defaults to red and can be changed through a command

    private enum AnimationType {
        None,
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
    }

    public static String strLEDColor = "";
    public static String strLEDAnimation = "";

    private static final int kSlot0StartIdx = 8;
    private static final int kSlot0EndIdx = 37;

    private static final int kSlot1StartIdx = 38;
    private static final int kSlot1EndIdx = 67; //67 OH MY GOD 67!!!!!! SO FUNNY

    private AnimationType m_anim0State = AnimationType.None;
    private AnimationType m_anim1State = AnimationType.None;

    private static AnimationType animation = AnimationType.None;

    private static AnimationType animation1 = AnimationType.None;

    private static AnimationType animation2 = AnimationType.None;

    private int ColorCycle = 0;
    private int AnimationCycle = 0;

    private final SendableChooser<AnimationType> m_anim0Chooser = new SendableChooser<AnimationType>();
    private final SendableChooser<AnimationType> m_anim1Chooser = new SendableChooser<AnimationType>();

    @Override
    public void periodic() {
        /* if the selection for slot 0 changes, change animations */
        final var anim0Selection = m_anim0Chooser.getSelected();
        if (m_anim0State != anim0Selection) {
            m_anim0State = anim0Selection;

            switch (m_anim0State) {
                default:
                strLEDAnimation = m_anim0State.toString();
                case ColorFlow:
                    m_candle.setControl(
                        new ColorFlowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                            .withColor(LEDColor)
                    );
                    break;
                case Rainbow:
                    m_candle.setControl(
                        new RainbowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                    );
                    break;
                case Twinkle:
                    m_candle.setControl(
                        new TwinkleAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                            .withColor(LEDColor)
                    );
                    break;
                case TwinkleOff:
                    m_candle.setControl(
                        new TwinkleOffAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                            .withColor(LEDColor)
                    );
                    break;
                case Fire:
                    m_candle.setControl(
                        new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                    );
                    break;
            }
        }

        /* if the selection for slot 1 changes, change animations */
        final var anim1Selection = m_anim1Chooser.getSelected();
        if (m_anim1State != anim1Selection) {
            m_anim1State = anim1Selection;

            switch (m_anim1State) {
                default:
                strLEDAnimation = m_anim1State.toString() + ", " + m_anim0State.toString();
                case Larson:
                    m_candle.setControl(
                        new LarsonAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                            .withColor(LEDColor)
                    );
                    break;
                case RgbFade:
                    m_candle.setControl(
                        new RgbFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                    );
                    break;
                case SingleFade:
                    m_candle.setControl(
                        new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                            .withColor(LEDColor)
                    );
                    break;
                case Strobe:
                    m_candle.setControl(
                        new StrobeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                            .withColor(LEDColor)
                    );
                    break;
                case Fire:
                    /* direction can be reversed by either the Direction parameter or switching start and end */
                    m_candle.setControl(
                        new FireAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                            .withDirection(AnimationDirectionValue.Backward)
                            .withCooling(0.4)
                            .withSparking(0.5)
                    );
                    break;
            }
        }
    }

    public void setLEDColor(RGBWColor color , boolean Cycle){
        if (Cycle) {
            ColorCycle += 1;
            if (ColorCycle > 7) {
                ColorCycle = 1;
            }

            if (ColorCycle == 1){
                color = new RGBWColor(frc.robot.subsystems.LEDs.LEDConstants.RED[0], frc.robot.subsystems.LEDs.LEDConstants.RED[1], frc.robot.subsystems.LEDs.LEDConstants.RED[2],0); //Red
            }
            else if (ColorCycle == 2){
                color = new RGBWColor(frc.robot.subsystems.LEDs.LEDConstants.ORANGE[0], frc.robot.subsystems.LEDs.LEDConstants.ORANGE[1], frc.robot.subsystems.LEDs.LEDConstants.ORANGE[2],0); //Orange
            }
            else if (ColorCycle == 3){
                color = new RGBWColor(frc.robot.subsystems.LEDs.LEDConstants.YELLOW[0], frc.robot.subsystems.LEDs.LEDConstants.YELLOW[1], frc.robot.subsystems.LEDs.LEDConstants.YELLOW[2],0); //Yellow
            }
            else if (ColorCycle == 4){
                color = new RGBWColor(frc.robot.subsystems.LEDs.LEDConstants.GREEN[0], frc.robot.subsystems.LEDs.LEDConstants.GREEN[1], frc.robot.subsystems.LEDs.LEDConstants.GREEN[2],0); //Green
            }
            else if (ColorCycle == 5){
                color = new RGBWColor(frc.robot.subsystems.LEDs.LEDConstants.BLUE[0], frc.robot.subsystems.LEDs.LEDConstants.BLUE[1], frc.robot.subsystems.LEDs.LEDConstants.BLUE[2],0); //Blue
            }
            else if (ColorCycle == 6){
                color = new RGBWColor(frc.robot.subsystems.LEDs.LEDConstants.PURPLE[0], frc.robot.subsystems.LEDs.LEDConstants.PURPLE[1], frc.robot.subsystems.LEDs.LEDConstants.PURPLE[2],0); //Purple
            }
            else if (ColorCycle == 7){
                color = new RGBWColor(frc.robot.subsystems.LEDs.LEDConstants.WHITE[0], frc.robot.subsystems.LEDs.LEDConstants.WHITE[1], frc.robot.subsystems.LEDs.LEDConstants.WHITE[2],0); //White
            }
        }
        LEDColor = color;
        m_candle.setControl(new SolidColor(255, 0).withColor(LEDColor));
    }

    public void setLEDAnimation(String animationString, boolean Cycle) { 
        if (Cycle) {
            AnimationCycle += 1;
            if (AnimationCycle > 10) {
                AnimationCycle = 1;
            }
            if (AnimationCycle == 1){
                animation = AnimationType.ColorFlow;
            }
            else if (AnimationCycle == 2){
                animation = AnimationType.Rainbow;
            }
            else if (AnimationCycle == 3){
                animation = AnimationType.Twinkle;
            }
            else if (AnimationCycle == 4){
                animation = AnimationType.TwinkleOff;
            }
            else if (AnimationCycle == 5){
                animation = AnimationType.Fire;
            }
            else if (AnimationCycle == 6){
                animation = AnimationType.Larson;
            }
            else if (AnimationCycle == 7){
                animation = AnimationType.RgbFade;
            }
            else if (AnimationCycle == 8){
                animation = AnimationType.SingleFade;
            }
            else if (AnimationCycle == 9){
                animation = AnimationType.Strobe;
            }
            else if (AnimationCycle == 10){
                animation = AnimationType.None;
            }
        else {
            if (animationString == "None") {
                animation = AnimationType.None;
            }
            else if (animationString == "ColorFlow") {
                animation = AnimationType.ColorFlow;
            }
            else if (animationString == "Rainbow") {
                animation = AnimationType.Rainbow;
            }
            else if (animationString == "Twinkle") {
                animation = AnimationType.Twinkle;
            }
            else if (animationString == "TwinkleOff") {
                animation = AnimationType.TwinkleOff;
            }
            else if (animationString == "Fire") {
                animation = AnimationType.Fire;
            }
            else if (animationString == "Larson") {
                animation = AnimationType.Larson;
            }
            else if (animationString == "RgbFade") {
                animation = AnimationType.RgbFade;
            }
            else if (animationString == "SingleFade") {
                animation = AnimationType.SingleFade;
            }
            
            else if (animationString == "Strobe") {
                animation = AnimationType.Strobe;
            }
        }

        }
         if (animation == AnimationType.ColorFlow || animation == AnimationType.Rainbow || animation == AnimationType.Twinkle || animation == AnimationType.TwinkleOff || animation == AnimationType.Fire){
                animation1 = animation;
            }
            else if (animation == AnimationType.Larson || animation == AnimationType.RgbFade || animation == AnimationType.SingleFade || animation == AnimationType.Strobe){
                animation2 = animation;
            }
            else {
                animation1 = AnimationType.None;
                animation2 = AnimationType.None;
        } //I made two different animation values just to piss off who ever is working with this (Good luck its your problem now, also Sam Bowling made it GRB instead of RGB so have fun with that)
    }

    public void StopLEDSubsystem() {
        setLEDColor(new RGBWColor(0, 0, 0, 0), false);
    }

    public String getLEDStats(){
        return LEDSubsystem.strLEDAnimation + ", " + LEDSubsystem.LEDColor;
    }

}

//Code made by Will Edwards(Freshman), help from Sam Bowling (Senior), Payton Gaultiey(Super Senior Mentor), Special thanks to: Jordan Shaw (Junior) for being Jordan.