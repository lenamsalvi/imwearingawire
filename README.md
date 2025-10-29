# I'm Wearing a Wire

A self-reflection audio recording device built on ESP32. Records voice memos to SD card and plays them back via Bluetooth headphones. Explores Bluetooth audio protocols (A2DP and HFP) in depth.

## Project Goals

- Deep exploration of Bluetooth Classic audio protocols
- Build a portable voice recording device for car-based self-reflection
- Learn ESP32 I2S audio processing and SD card storage
- Create a functional breadboard prototype before integration into physical device

## Hardware Components

### Core Components
- **MCU**: Inland ESP32 Development Board (ESP32-WROOM, Bluetooth Classic support)
- **Microphone**: ICS-43434 I2S MEMS Microphone (primary)
- **Microphone (backup)**: MAX4466 Electret Microphone Breakout
- **Storage**: MicroSD Card Breakout Board (FAT32 formatted with MBR)
- **Display**: Small OLED Screen (I2C)
- **Display (optional)**: E-ink screen (larger, for final build consideration)
- **Input**: 3 buttons with pull-down resistors on breadboard
- **Power**: USB (laptop-powered during development)

### Future Components (Not Yet Acquired)
- LiPo battery (700-1500mAh range)
- Battery charging module with USB-C
- Enclosure (design TBD)

## Pin Mappings

### ICS-43434 I2S Microphone
- WS (Word Select): GPIO ___
- SCK (Clock): GPIO ___
- SD (Data): GPIO ___
- VCC: 3.3V
- GND: GND

### SD Card (SPI)
- MISO: GPIO ___
- MOSI: GPIO ___
- CLK: GPIO ___
- CS: GPIO ___
- VCC: 3.3V
- GND: GND

### OLED Display (I2C)
- SDA: GPIO ___
- SCL: GPIO ___
- VCC: 3.3V
- GND: GND

### Buttons
- Button 1 (Record/Stop): GPIO ___
- Button 2 (Play/Pause): GPIO ___
- Button 3 (Menu/Select): GPIO ___

## Development Roadmap

### Phase 1: Environment Setup
- [x] Assemble hardware components
- [x] Format SD card (FAT32, MBR)
- [x] Wire buttons with pull-down resistors
- [x] Install ESP-IDF development environment (Windows)
- [x] Test basic ESP32 connectivity and upload test sketch

### Phase 2: Component Testing
- [ ] Wire and test OLED display (I2C communication)
- [ ] Wire and test SD card (FAT32 read/write operations)
- [ ] Wire and test ICS-43434 microphone (I2S audio capture)
- [ ] Test button inputs (debouncing and event handling)

### Phase 3: Core Functionality
- [ ] Implement WAV file recording (I2S mic → SD card)
- [ ] Implement file management (list, select, delete recordings)

### Phase 4: Bluetooth Audio Deep Dive
- [ ] Implement A2DP Bluetooth playback (SD card → headphones)
- [ ] Implement HFP/HSP Bluetooth mic input (headphone mic → ESP32)
- [ ] Handle Bluetooth pairing and connection management
- [ ] Test both profiles simultaneously

### Phase 5: Integration
- [ ] Build UI state machine for record/playback modes
- [ ] Implement OLED menu system with button controls
- [ ] Add audio level indicators and recording status
- [ ] Test complete workflow end-to-end

### Phase 6: Power and Enclosure (Future)
- [ ] Add battery and charging circuit
- [ ] Test power consumption and battery life
- [ ] Design and build enclosure
- [ ] Consider E-ink display swap for battery efficiency

## Technical Stack

**Development Framework**: ESP-IDF (switching from Arduino for better Bluetooth control)

**Key Protocols**:
- I2S (Inter-IC Sound) for digital microphone interface
- SPI for SD card communication
- I2C for OLED display
- A2DP (Advanced Audio Distribution Profile) for Bluetooth audio playback
- HFP/HSP (Hands-Free Profile/Headset Profile) for Bluetooth microphone input

**Audio Format**: WAV (uncompressed, 16-bit, configurable sample rate)

## Current Status

**Last Updated**: 2025-10-28

**Phase**: 1 - Environment Setup
**Current Task**: Installing ESP-IDF development environment

**Hardware Status**:
- ✓ All components acquired (except battery/charging)
- ✓ Buttons wired on breadboard
- ✓ SD card formatted
- ⧖ Components ready for wiring
- ⧖ Pin mappings to be determined during testing

## Development Notes

- Using breadboard prototype throughout development
- USB-powered from laptop until battery integration
- ICS-43434 chosen for superior I2S quality over analog MAX4466
- ESP-IDF required for full HFP Bluetooth stack (Arduino libraries insufficient)
- Will test individual components before integration

  **Phase 1 Completed**: 2025-10-29

  - ESP-IDF v5.5.1 installed to `C:\esp\` (custom path to avoid spaces in Windows username)
  - CP2102 USB-to-UART drivers required manual installation from Silicon Labs
  - ESP32 board confirmed working on COM3
  - Project location: `C:\esp\projects\imwearingawire\` (avoiding user folder path issues)
  - Blink test successful - toolchain verified working

## Resources

- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [ICS-43434 Datasheet](https://www.invensense.com/products/digital/ics-43434/)
- [Bluetooth A2DP Specification](https://www.bluetooth.com/specifications/specs/a2dp-1-3-2/)
- [Bluetooth HFP Specification](https://www.bluetooth.com/specifications/specs/hands-free-profile-1-8/)

## Project Structure

```
imwearingawire/
├── src/              # Source code
├── docs/             # Additional documentation
├── schematics/       # Circuit diagrams and layouts
└── README.md         # This file
```
