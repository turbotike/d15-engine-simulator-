// =============================================================================
//  Config.h - RCLC D15 board pins + global build config
// =============================================================================
//  Every pin number below was read off the rev3 board and checked pad-by-pad
//  against DESIGN.md's pin budget. Do not "tidy" them - the board is fabricated.
//
//  What changed from the Teensy board, and why it matters:
//    * Audio is two MAX98357A over I2S, not an SGTL5000 Audio Shield. There is
//      no codec to configure and no analog output level to set - digital full
//      scale IS the amp's full scale, so the old DC_LINEOUT_LEVEL games are gone.
//      The S3 has no internal DAC at all; Espressif removed it on this chip.
//    * ONE PCA9685 at 0x5F, lights only. Not two at 0x40/0x41.
//    * The servo bank is gone. The receiver drives the servos directly, so this
//      board never sees them.
//    * microSD is the S3's own SDMMC peripheral in 1-bit mode, not SPI.
//    * 8 MB of octal PSRAM, so sound packs live resident in RAM. The Teensy
//      build's streaming ladder (SdLoop) existed only because a 4.0 has 488 KB.
// =============================================================================
#pragma once
#include <Arduino.h>

// ---- I2C (PCA9685 + the accessory header) ----------------------------------
#define PIN_SDA             10
#define PIN_SCL             11
#define DC_I2C_HZ           400000

// ---- I2S out -> both MAX98357A ---------------------------------------------
//  Both amps share one 3-wire bus. Which channel each one plays is set in
//  HARDWARE by its SD_MODE resistor, not in software:
//      U4 / SPK1  R24 = 220k -> LEFT
//      U9 / SPK2  R25 = 1M   -> RIGHT
//  So the firmware just renders stereo and the board splits it.
#define PIN_I2S_BCLK        15
#define PIN_I2S_LRCK        16
#define PIN_I2S_DOUT        17

// ---- microSD, 1-bit SDMMC --------------------------------------------------
//  One bit, not four: sound packs stream at a few hundred KB/s at most and the
//  three saved pins were worth more than bandwidth nobody needs.
//  NOTE: the socket's card-detect contacts are NOT wired. A failed mount is the
//  only way this firmware can know there is no card in.
#define PIN_SD_CLK          18
#define PIN_SD_CMD          8
#define PIN_SD_DAT0         9

// ---- analog sense ----------------------------------------------------------
//  Both on ADC1, which is the only ADC that still works with the radio up.
#define PIN_VBAT_SENSE      4        // ADC1_CH3, R11/R12 = 100k/47k
#define PIN_BEC_SENSE       5        // ADC1_CH4, R13/R14 = 100k/100k
#define DC_VBAT_RATIO       (147.0f / 47.0f)   // node volts -> pack volts
#define DC_BEC_RATIO        2.0f
// Calibrate once against a meter and keep the offset; the S3's ADC reference
// varies part to part by enough to matter on a low-battery decision.
#define DC_VBAT_TRIM        1.000f

// ---- RC input --------------------------------------------------------------
//  One firmware, three protocols, auto-detected: iBUS (115200 8N1), SBUS
//  (100k 8E2, inverted or not), else classic PWM on the CH headers.
#define PIN_RC_SERIAL_RX    12       // RX header, series 330R (R10)
#define RC_SERIAL           Serial1 // remapped onto PIN_RC_SERIAL_RX in begin()
#define RC_FORCE_MODE       0        // 0=auto 1=iBUS 2=SBUS 3=PWM

//  PWM mode: eight channel headers, one per channel.
#define PIN_CH1             13
#define PIN_CH2             14
#define PIN_CH3             21
#define PIN_CH4             38
#define PIN_CH5             39
#define PIN_CH6             40
#define PIN_CH7             41
#define PIN_CH8             42
#define DC_PWM_CHANNELS     8

// ---- ESC out ---------------------------------------------------------------
#define PIN_ESC             47
#define DC_ESC_LEDC_CH      0
#define DC_ESC_LEDC_HZ      50
#define DC_ESC_LEDC_BITS    14   // S3 LEDC maxes at 14 bits (SOC_LEDC_TIMER_BIT_WIDE_NUM);
                                 // the original ESP32 allowed 20. At 50 Hz this is still
                                 // 1.22 us per count - far finer than the 12 us slew step.

// ---- accessory header spare GPIO -------------------------------------------
#define PIN_ACC_IO1         6
#define PIN_ACC_IO2         7

// ---- channel map (0-based), same defaults as the proven truck build --------
#define CH_STEERING         0
#define CH_THROTTLE         1        // >centre = throttle, <centre = brake
#define CH_GEAR             3
#define CH_BRAKE            4
#define CH_ENGINE           5
#define CH_LIGHTS           6
#define CH_HORN             7
#define CH_JAKE             8
#define CH_AUX              9

// ---- lights: ONE PCA9685, ten channels -------------------------------------
//  0x5F, not 0x41. Rev3 of the board grounds the chip's VSS through a spare
//  ULN2803 output, which forced A1-A4 high and A5 low:
//      address = 1 A5 A4 A3 A2 A1 A0 = 1 0 1 1 1 1 1 = 0x5F
//  If the board is ever respun with a proper ground via for VSS, this goes
//  back to 0x41.
#define DC_PCA_ADDR         0x5F
//  ~600 Hz. The servos are gone, so nothing on this board wants 50 Hz any more,
//  and an LED dimmed at 50 Hz visibly flickers.
#define DC_PCA_HZ           600.0f

//  Channel 15 is NOT a light. Its ULN2803 output (U6 pin 11) is the track that
//  grounds the PCA9685. Driving it is harmless - it shorts ground to ground
//  through the Darlington - but the channel is spoken for.
#define DC_PCA_CH_RESERVED  15

// ---- sim base (tune in EngineSim.h) ---------------------------------------
#define DC_IDLE_RPM         680.0f
#define DC_MAX_RPM          2100.0f

// ---- audio -----------------------------------------------------------------
//  *** BOARD LIMITATION - READ THIS BEFORE CHASING VOLUME IN SOFTWARE ***
//
//  U4/U9 pin 2 (GAIN_SLOT) is UNCONNECTED on rev3. Per the MAX98357A datasheet
//  that selects the 9 dB setting - the second LOWEST of five:
//
//      100k to GND  15 dB      <- the maximum, what this board should have had
//      direct GND   12 dB
//      unconnected   9 dB      <- rev3, by omission
//      direct VDD    6 dB
//      100k to VDD   3 dB
//
//  The DIYGuy board this replaces uses a PAM8403 at a FIXED 24 dB. So rev3 is
//  15 dB down on the thing it was meant to beat, and no amount of digital drive
//  recovers it - the shortfall is in the amp's own gain stage, downstream of
//  everything the firmware can touch. DESIGN.md said "GAIN floating gives 9 dB"
//  and that was carried into the footprint without asking whether 9 dB was
//  enough to replace a 24 dB amp. It is not.
//
//  REWORK: 100k from pin 2 to GND on both U4 and U9 -> 15 dB (+6 dB).
//  Or bridge pin 2 to pin 3 (adjacent) for 12 dB (+3 dB) with no part.
//  REV4: route GAIN_SLOT to a 100k-to-GND pad.

#define DC_OUT_RATE         44100.0f
#define DC_AUDIO_FRAMES     256      // frames per render block
#define DC_AUDIO_CORE       0        // render pinned off the core running WiFi
// 0.92 came from the Teensy's SGTL5000 dacVolume, where an analog stage sat
// after it. On a MAX98357A full-scale PCM IS full output and nothing downstream
// can clip, so a digital master belongs at unity and should only come down when
// the user turns it down.
#define DC_MASTER_VOL       1.00f

// ---- debug -----------------------------------------------------------------
#define DC_DEBUG            1
