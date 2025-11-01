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
- WS (Word Select): GPIO 25
- SCK (Clock): GPIO 26
- SD (Data): GPIO 27
- VCC: 3.3V
- GND: GND

### SD Card (SPI)
- MISO: GPIO 19
- MOSI: GPIO 23
- CLK: GPIO 18
- CS: GPIO 5
- VCC: 3.3V
- GND: GND

### OLED Display (I2C)
- SDA: GPIO 21
- SCL: GPIO 22
- VCC: 3.3V
- GND: GND
- Address: 0 x 3D

### Buttons
- Button Left (Previous Screen): GPIO 32
- Button Middle (Record/Select): GPIO 33
- Button Right (Next Screen): GPIO 34

## Development Roadmap

### Phase 1: Environment Setup
- [x] Assemble hardware components
- [x] Format SD card (FAT32, MBR)
- [x] Wire buttons with pull-down resistors
- [x] Install ESP-IDF development environment (Windows)
- [x] Test basic ESP32 connectivity and upload test sketch

### Phase 2: Component Testing
- [x] Wire and test OLED display (I2C communication)
- [x] Wire and test SD card (FAT32 read/write operations)
- [x] Wire and test ICS-43434 microphone (I2S audio capture)
- [x] Test button inputs (debouncing and event handling)

### Phase 3: Core Functionality
- [x] Implement WAV file recording (I2S mic → SD card)
- [ ] Debug and optimize audio quality (sample rate, I2S configuration)
- [ ] Implement file management (list, select, delete recordings)
- [ ] Add persistent filename counter across reboots

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

**Last Updated**: 2025-10-30

**Phase**: 3 - Core Functionality (Audio Quality Debugging)

**Current Task**: Diagnosing I2S audio quality issues (choppy playback, speed inconsistency)

**Hardware Status**:
- All components wired and functional on breadboard
- I2S microphone capturing audio (quality issues under investigation)
- SD card recording WAV files successfully
- OLED display showing 6-screen navigation UI
- Button-based recording control working

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

**Phase 2 Completed**: 2025-10-30
- OLED display (SSD1306) initialized via I2C at 400kHz on GPIO 21/22
- SD card mounted via SPI at 4MHz (initially 400kHz for breadboard stability)
- Button debouncing implemented (200ms) with pull-down resistors
- 6-screen navigation UI implemented (Title, SD Status, Record, Playback, Settings, Graphics)
- I2S microphone initialized on GPIO 25/26/27 in Philips standard mode
- Full WAV recording pipeline: I2S capture → fwrite to SD → WAV header generation
- Real-time audio level visualization on OLED during recording
- Automatic filename generation (memo_001.wav, memo_002.wav, etc.)
- Power management: I2S disabled when not recording
- Error handling added for SD card writes and WAV header generation

**Code Quality Improvements** (Post-Phase 2 Review):
- Fixed OLED display bug: removed text overlapping audio level bar
- Fixed SD card space allocation: removed hardcoded 16KB cluster size causing mount issues
- Added comprehensive error checking: all fwrite operations now verify bytes written
- Increased SD SPI speed: 400kHz → 4MHz for better write performance
- Implemented power management: I2S channel now disabled between recordings
- Enhanced error reporting: SD operations update on-screen status messages

## Resources

- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [ICS-43434 Datasheet](https://www.invensense.com/products/digital/ics-43434/)
- [Bluetooth A2DP Specification](https://www.bluetooth.com/specifications/specs/a2dp-1-3-2/)
- [Bluetooth HFP Specification](https://www.bluetooth.com/specifications/specs/hands-free-profile-1-8/)
- [Inland Pin Map](https://community.microcenter.com/kb/articles/652-inland-esp32-core-board-black-and-eco-friendly)

## Known Issues and Active Debugging

### Audio Quality Issues (Current Focus)
- **Symptom**: Recorded audio plays back choppy and speed-inconsistent
- **Suspected causes**:
  - I2S clock configuration mismatch with ICS-43434 expectations
  - Sample rate mismatch between capture and WAV header
  - Potential SD write speed issues on breadboard wiring
  - I2S buffer configuration (1024 samples, 50ms polling may be insufficient)
- **Investigation approach**:
  - Verify actual sample rate of captured audio
  - Test with MAX4466 analog microphone as baseline
  - Measure I2S timing with logic analyzer (future)
  - Increase I2S buffer size and reduce polling interval

### Filename Persistence Issue
- **Symptom**: Recording counter resets to memo_001.wav on every reboot
- **Cause**: Counter stored in RAM, not persisted to storage
- **Impact**: Each reboot overwrites previous recordings
- **Planned fix**: Scan SD card at boot to find highest memo number + 1

### SD Card Hot-Swap Issue
- **Symptom**: Removing/reinserting SD card without reboot causes recording failure
- **Cause**: Mount happens once at startup, `sd_initialized` flag doesn't update on card removal
- **Impact**: Silent recording failures with no user feedback
- **Not critical**: Device is embedded, card shouldn't be hot-swapped in normal use

## Design Decisions Under Consideration

### Microphone Selection
- **Current**: ICS-43434 I2S MEMS microphone
- **Alternative**: MAX4466 electret microphone with analog ADC input
- **Considerations**:
  - I2S offers higher quality ceiling but requires precise clock configuration
  - Analog ADC is simpler, more forgiving of breadboard wiring
  - For voice memos, analog quality may be "good enough"
  - Decision pending audio quality debugging results

### Timestamp vs Counter Filenames
- **Current**: Sequential counter (memo_001.wav, memo_002.wav)
- **Future Option 1**: ESP32 internal RTC (resets on power loss)
- **Future Option 2**: External RTC module (DS3231) with battery backup
- **Considerations**:
  - Counter requires SD card scanning at boot for persistence
  - Internal RTC requires initial time-setting mechanism (no WiFi in car)
  - External RTC adds $3-5 cost and one I2C device but provides true timestamps
  - Decision: Counter for Phase 3, consider RTC in Phase 6 (enclosure/battery)

## Technical Implementation Details

### Audio Recording Pipeline
1. **I2S Configuration**: 16kHz sample rate, 16-bit mono, Philips standard mode
2. **Capture Loop**: 50ms polling, 1024-sample buffer (64ms at 16kHz)
3. **WAV Format**: Standard PCM WAV with 44-byte header
4. **Storage**: Direct fwrite to SD card with error checking
5. **Power**: I2S channel disabled between recordings

### Code Architecture
- **State Machine**: 6-screen UI with button-driven navigation
- **Recording State**: Boolean flags with file handle management
- **Error Handling**: All SD operations check return values and update status display
- **Concurrency**: Single-threaded main loop, no RTOS tasks yet

### Performance Characteristics
- **SD Write Speed**: 4MHz SPI (32 KB/s theoretical, actual varies)
- **Audio Data Rate**: 16kHz × 16-bit × 1 channel = 32 KB/s
- **Critical timing**: Write speed must match or exceed audio data rate to avoid dropouts

## Project Structure

```
imwearingawire/
├── main/                    # ESP-IDF main component (source code)
│   ├── oled_test.c         # Main application: UI, recording, I2S, SD card
│   ├── CMakeLists.txt      # Component build configuration
│   └── idf_component.yml   # Component dependencies (ssd1306 driver)
├── build/                   # Build output (generated, not in git)
├── sdkconfig.defaults       # ESP-IDF project configuration
├── CMakeLists.txt          # Root project configuration
└── README.md               # This file
```
