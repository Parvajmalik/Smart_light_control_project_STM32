# Smart_light_control_project_STM32
Design a smart light control system that automatically adjusts LED brightness based on ambient light levels.The system will display real-time ambient light intensity and LED brightness level on an I2C-based HD44780 LCD

𝐏𝐫𝐨𝐣𝐞𝐜𝐭 𝐎𝐯𝐞𝐫𝐯𝐢𝐞𝐰:
 
1. Ambient Light Detection: Use an LDR connected to an ADC pin on the STM32F407 to measure ambient light levels. Based on the LDR reading, the system will adjust the brightness of an LED (connected via PWM).
2. User Interface: The HD44780 LCD, interfaced via I2C, will display the current ambient light level and LED brightness percentage.
3. Communication: Use UART communication to send real-time data to a terminal or a PC for monitoring and logging purposes.
4. Threshold Control: Set thresholds for different brightness levels, so the LED adjusts smoothly as ambient light changes.
5. Manual Override: Add buttons for manual LED control, allowing the user to turn on or off the LED irrespective of ambient light conditions.

𝐂𝐨𝐦𝐩𝐨𝐧𝐞𝐧𝐭𝐬 𝐚𝐧𝐝 𝐏𝐞𝐫𝐢𝐩𝐡𝐞𝐫𝐚𝐥𝐬:
 
• STM32F407 Discovery Board (main controller)
• LDR (connected to ADC input pin)
• External LED (connected to a PWM pin)
• HD44780 LCD (connected via I2C)
• Two Button (for manual override ON/OFF, connected to GPIO pin)

𝐏𝐫𝐨𝐣𝐞𝐜𝐭 𝐈𝐦𝐩𝐥𝐞𝐦𝐞𝐧𝐭𝐚𝐭𝐢𝐨𝐧 𝐒𝐭𝐞𝐩𝐬:
 
1. Ambient Light Sensing:
     • Set up the LDR as a voltage divider and connect it to an ADC pin.
     • Use ST-HAL ADC functions to measure the analog value representing ambient light intensity.
2. LED Brightness Control:
     • Use PWM (Pulse Width Modulation) to control the brightness of the LED.
     • Adjust PWM duty cycle based on the LDR reading.
3. Display Data on LCD:
     • Interface the HD44780 LCD with the STM32F407 using the I2C protocol.
     • Display real-time values, such as ambient light level and LED brightness percentage.
4. Communication via UART:
     • Configure UART to send ambient light and brightness data to a connected terminal or PC.
     • Log data for later analysis.
5. Manual Control:
     • Use buttons to override the automatic brightness adjustment.
     • When pressed, a button LED will be ON, pressing again will be OFF, and other button to switch back to ambient light control mode.
6. Code Structure:  Main Program: Initializes peripherals, reads LDR values, updates LED brightness, displays data on LCD, and handles UART communication.
     • Interrupt Handlers: Use external interrupt for the button press to toggle manual override.
     • Polling/Interrupt for ADC: Regularly poll the ADC or set up an interrupt to sample the LDR values periodically.
 
This project combines embedded systems concepts like ADC, PWM, GPIO, I2C, UART, and integrates a real-world application of smart lighting.

