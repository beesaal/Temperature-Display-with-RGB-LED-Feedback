### **Overview**
This project demonstrates the integration of an MCP7000A temperature sensor with an OLED display and an RGB LED to visually indicate temperature ranges. The system reads the temperature data from the sensor, displays it on the OLED, and uses PWM signals to control the RGB LED, changing its color based on the current temperature.

### **Features**

**1.	Temperature Measurement:**
        
- [ ] Utilizes the MCP7000A temperature sensor to measure ambient temperature.
- [ ] Reads temperature data and updates in real-time.

**2.	OLED Display:**

- [ ] Displays the current temperature in the top right corner of the OLED screen.
- [ ] Shows the temperature with one digit after the decimal place for precision.

**3.	RGB LED Indicator:**

- [ ] Illuminates the RGB LED with different colors based on temperature ranges.
- [ ] Uses PWM signals to achieve smooth color transitions and optimal color mixing.


###**Temperature Ranges and Corresponding LED Colors**

- [ ] -15°C to 5.0°C:------------------- Blue
- [ ] 5.1°C to 15.0°C:------------------ Yellow
- [ ] 15.1°C to 25.0°C:----------------- Orange
- [ ] Above 25.1°C:-------------------- Red

### **Implementation Details**
•	PWM Control:

- [ ] Adjusts the period of the PWM output to control the intensity of the red, green, and blue components of the RGB LED.
- [ ] Ensures precise color mixing for accurate representation of the temperature range.

•	Real-time Display:

- [ ] Continuously updates the OLED display with the latest temperature reading.
- [ ] Ensures that the displayed temperature is always current and accurate.


### **Hardware Requirements**

- MCP7000A Temperature Sensor
- OLED Display
- RGB LED
- STM32-NUCLEO-L432KC
- Power supply and necessary connectors

### **Software Requirements**

- Embedded C
- PWM control libraries
- OLED display libraries


### **Code Repository**
The complete source code, including setup instructions and schematics, is available in this GitHub repository. This project is an excellent demonstration of basic embedded system integration, providing practical experience in sensor interfacing, display control, and PWM signal manipulation.
