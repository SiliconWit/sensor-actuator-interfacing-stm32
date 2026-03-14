---
title: "Sensor and Actuator Interfacing with STM32 - Collaboration Guide"
description: "Contributing guide for Sensor and Actuator Interfacing with STM32 course content"
tableOfContents: true
sidebar:
  order: 999
---

# Sensor and Actuator Interfacing with STM32

![Build](https://img.shields.io/badge/build-passing-brightgreen)
![License](https://img.shields.io/badge/license-MIT-blue)
![Contributors Welcome](https://img.shields.io/badge/contributors-welcome-orange)

**Read this course at:** [https://siliconwit.com/education/sensor-actuator-interfacing-stm32/](https://siliconwit.com/education/sensor-actuator-interfacing-stm32/)

A hands-on course covering sensor, actuator, and communication module interfacing on the STM32 Blue Pill (STM32F103C8T6) using STM32CubeIDE and HAL. Topics include GPIO, ADC, PWM, I2C, SPI, UART, RFID, stepper motors, DMA, CAN bus, and a multi-sensor capstone project.

## Lessons

| # | Title |
|---|-------|
| 1 | GPIO and Digital Interfacing |
| 2 | ADC and Analog Signal Conditioning |
| 3 | PWM, Timers, and Motor Control |
| 4 | I2C Protocol: Sensors and Displays |
| 5 | SPI Protocol: Storage and Displays |
| 6 | UART Devices: GPS, Bluetooth, and RS-485 |
| 7 | RFID, NFC, and Identification Systems |
| 8 | Stepper Motors and Encoder Feedback |
| 9 | DMA, Interrupts, and CAN Bus |
| 10 | Capstone: Multi-Sensor Data Logger |

## File Structure

```
sensor-actuator-interfacing-stm32/
├── index.mdx
├── gpio-digital-interfacing.mdx
├── adc-analog-signal-conditioning.mdx
├── pwm-timers-motor-control.mdx
├── i2c-sensors-displays.mdx
├── spi-storage-displays.mdx
├── uart-gps-bluetooth-rs485.mdx
├── rfid-nfc-identification.mdx
├── stepper-motors-encoders.mdx
├── dma-interrupts-can-bus.mdx
├── multi-sensor-data-logger.mdx
└── README.md
```

## How to Contribute

1. Fork the repository: [SiliconWit/sensor-actuator-interfacing-stm32](https://github.com/SiliconWit/sensor-actuator-interfacing-stm32)
2. Create a feature branch: `git checkout -b feature/your-topic`
3. Make your changes and commit with a clear message
4. Push to your fork and open a Pull Request against `main`
5. Describe what you changed and why in the PR description

## Content Standards

- All lesson files use `.mdx` format
- Do not use `<BionicText>` in this course
- Code blocks should include a title attribute:
  ````mdx
  ```c title="main.c"
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  ```
  ````
- Use Starlight components (`<Tabs>`, `<TabItem>`, `<Steps>`, `<Card>`) where appropriate
- Keep paragraphs concise and focused on practical application
- Include working code examples that readers can compile and flash
- All pin assignments must be valid for the STM32F103C8T6

## Local Development

Clone the main site repository and initialize submodules:

```bash
git clone --recurse-submodules <main-repo-url>
cd siliconwit-com
npm install
npm run dev
```

To test a production build:

```bash
npm run build
```

## License

This course content is released under the [MIT License](LICENSE).
