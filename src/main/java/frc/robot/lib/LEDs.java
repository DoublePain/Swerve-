package frc.robot.lib;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDs {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    /**
     * Number of LEDs in the strip. Change this number to match the number of LEDs
     */
    private int m_numLEDs = 70;
    private int m_r, m_g, m_b;
    private int m_midTop = 48;
    private int[] mapL = {48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,3,4,5,6,7,8,9,10,11,12,13,14,15};
    private int m_meteorCntR = 0;
    private int m_meteorCntL = 0;

    private int m_meteorTimeCnt = 0;
    private Color8Bit m_allianceColor1 = null;
    private Color8Bit m_allianceColor2= null;
    private Color8Bit m_allianceColor3 = null;
    private Color8Bit m_allianceColor4 = null;
    private boolean fadeDir = true;
    private int fadeCnt = 0;

    /**
     * This is a constructor for the class. It has the same name as the class and
     * has no return type.
     *
     * @param _port The PWM port the LEDs are plugged into
     */
    public LEDs(int _port) {
        m_led = new AddressableLED(_port);
        m_ledBuffer = new AddressableLEDBuffer(m_numLEDs);
        m_led.setLength(m_ledBuffer.getLength());
        setAllianceColor();
        updateLEDs();

    }

    public void updateLEDs() {
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    /**
     * Set all the LEDs to the requested RGB values
     *
     * @param _r Red Value between 0-255
     * @param _g Green Value between 0-255
     * @param _b Blue Value between 0-255
     */
    public void setRGBColor(int _r, int _g, int _b) {
        if(m_r != _r || m_g != _g || m_b != _b){
            for (int i = 0; i < m_numLEDs; i++) {
                m_ledBuffer.setRGB(i, _r, _g, _b);
            }
            m_r = _r; m_g = _g; m_b = _b;
            updateLEDs();
        }
        
    }
    public void meteor(){

        if(++m_meteorTimeCnt > 4){
            m_meteorTimeCnt = 0;
            if(m_meteorCntL > mapL.length-1){ 
                m_meteorCntL = 0;
            }
            clearLEDBuffer();
            m_ledBuffer.setLED(mapL[m_meteorCntL]-0, m_allianceColor1);
            m_ledBuffer.setLED(mapL[m_meteorCntL]-1, m_allianceColor2);
            m_ledBuffer.setLED(mapL[m_meteorCntL]-2, m_allianceColor3);
            m_ledBuffer.setLED(mapL[m_meteorCntL]-3, m_allianceColor4);
            if(m_meteorCntR > 34){
                m_meteorCntR = 0;
            }
         
            m_ledBuffer.setLED(m_midTop-m_meteorCntR-1, m_allianceColor1);
            m_ledBuffer.setLED(m_midTop-m_meteorCntR, m_allianceColor2);
            m_ledBuffer.setLED(m_midTop-m_meteorCntR+1, m_allianceColor3);
            m_ledBuffer.setLED(m_midTop-m_meteorCntR+2, m_allianceColor4);

            m_meteorCntR++;
            m_meteorCntL++;
            updateLEDs();
        }
    }
    public void fade(){

        if(++fadeCnt < 100){
            if(fadeDir){
                if(GD.G_Alliance == Alliance.Red){
                    setRGBColor(100-fadeCnt, 0, 0);
                }else {
                    setRGBColor(0, 0, 100-fadeCnt);
                }
            }else {
                if(GD.G_Alliance == Alliance.Red){
                    setRGBColor(fadeCnt, 0, 0);
                }else {
                    setRGBColor(0, 0, fadeCnt);
                }
            }
         }else {
             fadeDir = !fadeDir;
             fadeCnt = 0;
         }
    }
    /**
     * Set all the LEDs to the Color8Bit value
     *
     * @param _color The color to set
     */
    public void setRGBColor(Color8Bit _color) {
        setRGBColor(_color.red, _color.green, _color.blue);
    }

    /**
     * Set the LEDs to the color of the alliance the team is on.
     * If the the robot is not connected to the Field Management System (FMS)
     * then the color is green.
     */
    public void setAllianceColor() {
        if (GD.G_Alliance == Alliance.Blue) {
            setRGBColor(0, 0, 100);
            m_allianceColor1 = new Color8Bit(0, 0, 180);
            m_allianceColor2 = new Color8Bit(0, 0, 80);
            m_allianceColor3 = new Color8Bit(0, 0, 40);
            m_allianceColor4 = new Color8Bit(0, 0, 10);


        } else {
            setRGBColor(100, 0, 0);
            m_allianceColor1 = new Color8Bit(180, 0, 0);
            m_allianceColor2 = new Color8Bit(80, 0, 0);
            m_allianceColor3 = new Color8Bit(40, 0, 0);
            m_allianceColor4 = new Color8Bit(10, 0, 0);

        }
    }
    public void clearLEDBuffer(){
        for (int i = 0; i < m_numLEDs; i++){
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        
    }
}