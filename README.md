🚀 ESP32-S3 I2C Low-Level Driver

A battle-tested, low-level I2C driver for the ESP32-S3 microcontroller, engineered for robust communication with I2C devices like EEPROMs (e.g., AT24C02). Built with Embedded C and FreeRTOS, this driver demonstrates cutting-edge embedded systems programming, delivering precision, reliability, and scalability for IoT applications. Think of it as the arc reactor of I2C communication—compact, powerful, and meticulously crafted. 📋 Table of Contents

🛠️ About the Project ✨ Features 🧰 Technologies Used 📂 Project Structure ⚙️ Installation 🚀 Usage 🧪 Testing 🔧 Challenges Overcome 🌟 Future Enhancements 🤝 Contributing 📬 Contact

🛠️ About the Project This project is a high-performance I2C driver for the ESP32-S3, designed to interface seamlessly with I2C peripherals like EEPROMs. By leveraging low-level register manipulation, interrupt-driven communication, and FreeRTOS synchronization, it ensures reliable data transfers in real-time environments. The driver includes an I2C bus scanner and EEPROM read/write functionality, showcasing practical applications in embedded systems. Why this project shines:

Demonstrates mastery of low-level hardware programming and real-time systems. Implements robust error handling for mission-critical reliability. Built with ESP-IDF, the industry-standard framework for ESP32 development. Showcases my passion for crafting efficient, scalable solutions for IoT and embedded systems.

This project is a testament to my ability to tackle complex hardware challenges with elegance and precision, making it a standout piece for recruiters seeking top-tier embedded engineers. ✨ Features

Precision I2C Control: Direct manipulation of ESP32-S3 I2C registers for fine-tuned communication. EEPROM Compatibility: Supports read/write operations for EEPROMs (e.g., AT24C02) with configurable 1-byte or 2-byte addressing. I2C Bus Scanner: Scans addresses (0x08–0x77) to detect connected devices effortlessly. Robust Error Handling: Implements retries (up to 3 attempts) and bus reset for fault tolerance. Interrupt-Driven Design: Uses FreeRTOS semaphores and interrupts for low-latency, efficient transactions. Flexible Configuration: Customizable SDA/SCL pins, clock speeds (e.g., 100 kHz), and page sizes.

🧰 Technologies Used

Language: Embedded C Platform: ESP32-S3 microcontroller Framework: ESP-IDF (Espressif IoT Development Framework) RTOS: FreeRTOS for real-time task synchronization Hardware: I2C peripherals (e.g., AT24C02 EEPROM) Tools: CMake, Git, ESP-IDF toolchain

📂 Project Structure

image
⚙️ Installation Get this driver up and running on your ESP32-S3 with these steps:

Install ESP-IDF:

Follow the ESP-IDF Getting Started Guide to set up the ESP-IDF toolchain.

Clone the Repository: git clone https://github.com/your-username/ESP32-I2C-Driver.git cd ESP32-I2C-Driver

Configure the Project:

Run the ESP-IDF configuration tool:idf.py menuconfig

Verify I2C settings (SDA pin 9, SCL pin 8, 100 kHz clock speed) match your hardware.

Build the Project: idf.py build

Flash to ESP32-S3:

Connect your ESP32-S3 board via USB. Flash the firmware (replace /dev/ttyUSB0 with your port):idf.py -p /dev/ttyUSB0 flash

Monitor Output: idf.py monitor

🚀 Usage The main.c file provides a ready-to-run example that:

Initializes the I2C driver with SDA (pin 9), SCL (pin 8), and 100 kHz clock speed. Scans the I2C bus to detect devices (addresses 0x08–0x77). Writes 4 bytes (0x12, 0x34, 0x56, 0x78) to an EEPROM at address 0x50. Reads 4 bytes from the EEPROM and logs the results.

Customize the Application:

Edit main.c to adjust I2C parameters (e.g., pins, clock speed). Use i2c_low_level_write and i2c_low_level_read for custom I2C transactions. Leverage i2c_eeprom_page_write for efficient large data writes.

Example Log Output: image

🧪 Testing

Hardware Setup: Connect an I2C device (e.g., AT24C02 EEPROM) to SDA (pin 9) and SCL (pin 8) with 4.7kΩ pull-up resistors. I2C Scanner: Detects devices on the bus, logging their addresses. EEPROM Test: Verifies read/write functionality with a 4-byte test sequence. Error Handling: Retries failed transactions (up to 3 attempts) and resets the bus on errors.

🔧 Challenges Overcome

NACK Error Handling: Implemented a retry mechanism and bus reset to handle I2C NACK errors, ensuring robust communication. Interrupt Optimization: Fine-tuned interrupt-driven transactions with FreeRTOS semaphores to minimize latency and ensure reliability. Register-Level Precision: Mastered ESP32-S3 I2C register manipulation for precise control, avoiding high-level driver abstractions. EEPROM Compatibility: Designed flexible addressing (1-byte or 2-byte) to support various I2C devices.

🌟 Future Enhancements

Unit Testing: Integrate a test suite using Unity or CTest to validate driver functionality. Multi-Device Support: Extend the driver to handle multiple I2C devices concurrently. Dynamic Clock Scaling: Add support for dynamic clock speed adjustments based on device requirements. Documentation: Generate Doxygen documentation for detailed code insights.

🤝 Contributing Contributions are welcome to make this driver even more powerful! To contribute:

Fork the repository. Create a new branch (git checkout -b feature-name). Make your changes and commit (git commit -m "Add feature-name"). Push to the branch (git push origin feature-name). Open a pull request with a clear description of your changes.

Please adhere to ESP-IDF coding standards and include detailed comments. 📬 Contact

Name: Gaurav Kumar Email: gkumar20112000@gmail.com LinkedIn: www.linkedin.com/in/gaurav-kumar-b89570317

Ready to collaborate or discuss embedded systems? Reach out—I’m always up for a challenge!

Built with precision, passion, and a touch of genius. Let’s power the future of IoT together!
