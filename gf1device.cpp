/* GF1 device class - Version 1.0.0
   Requires CP2130 class version 1.1.0 or later
   Copyright (c) 2022 Samuel Lourenço

   This library is free software: you can redistribute it and/or modify it
   under the terms of the GNU Lesser General Public License as published by
   the Free Software Foundation, either version 3 of the License, or (at your
   option) any later version.

   This library is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
   License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this library.  If not, see <https://www.gnu.org/licenses/>.


   Please feel free to contact me via e-mail: samuel.fmlourenco@gmail.com */


// Includes
#include <cmath>
#include <sstream>
#include <unistd.h>
#include <vector>
#include "gf1device.h"

// Definitions
const uint8_t EPOUT = 0x01;      // Address of endpoint assuming the OUT direction
const uint8_t FSTARTLSB = 0xc0;  // Mask for the Fstart LSBs register
const uint8_t FSTARTMSB = 0xd0;  // Mask for the Fstart MSBs register

// Amplitude conversion constants
const uint AQUANTUM = 255;  // Quantum related to the 8-bit resolution of the AD5160 SPI potentiometer

// Frequency conversion constants
const uint FQUANTUM = 16777216;  // Quantum related to the 24-bit frequency resolution of the AD5932 waveform generator
const float MCLK = 50000;        // 50MHz clock

// Private convenience function that is used to clear the signals going to the CTRL and INTERRUPT pins on the AD5932 waveform generator
void GF1Device::clearCtrlInterrupt(int &errcnt, std::string &errstr)
{
    cp2130_.setGPIO2(false, errcnt, errstr);  // Set GPIO.2 low (corresponds to the CTRL pin)
    cp2130_.setGPIO3(false, errcnt, errstr);  // Set GPIO.3 low (corresponds to the INTERRUPT pin)
}

// Private convenience function used to toggle the signal going to the CTRL pin on the AD5932 waveform generator
void GF1Device::toggleCtrl(int &errcnt, std::string &errstr)
{
    cp2130_.setGPIO2(true, errcnt, errstr);  // Set GPIO.2 to a logical high
    cp2130_.setGPIO2(false, errcnt, errstr);  // and then to a logical low
}

// Private convenience function used to toggle the signal going to the INTERRUPT pin on the AD5932 waveform generator
void GF1Device::toggleInterrupt(int &errcnt, std::string &errstr)
{
    cp2130_.setGPIO3(true, errcnt, errstr);  // Set GPIO.3 to a logical high
    cp2130_.setGPIO3(false, errcnt, errstr);  // and then to a logical low
}
    
GF1Device::GF1Device() :
    cp2130_()
{
}

// Diagnostic function used to verify if the device has been disconnected
bool GF1Device::disconnected() const
{
    return cp2130_.disconnected();
}

// Checks if the device is open
bool GF1Device::isOpen() const
{
    return cp2130_.isOpen();
}

// Sets the frequency and amplitude of the generated signal to zero, and sets its waveform to sinusoidal
void GF1Device::clear(int &errcnt, std::string &errstr)
{
    clearCtrlInterrupt(errcnt, errstr);  // Clear "CTRL" and "INTERRUPT" signals
    cp2130_.selectCS(0, errcnt, errstr);  // Enable the chip select corresponding to channel 0, and disable any others
    std::vector<uint8_t> clearFrequency = {
        0x0f, 0xdf,              // Sinusoidal waveform, automatic increments, MSBOUT pin enabled, SYNCOUT pin enabled, B24 = 1, SYNCSEL = 1
        0x10, 0x00,              // Zero frequency increments
        0x20, 0x00, 0x30, 0x00,  // Delta frequency set to zero
        0x40, 0x00,              // Increment interval set to zero
        0xc0, 0x00, 0xc0, 0x00   // Start frequency set to zero
    };
    cp2130_.spiWrite(clearFrequency, EPOUT, errcnt, errstr);  // Set the waveform to sinusoidal and the frequency to zero (AD5932 on channel 0)
    usleep(100);  // Wait 100us, in order to prevent possible errors while disabling the chip select (workaround)
    cp2130_.selectCS(1, errcnt, errstr);  // Enable the chip select corresponding to channel 1, and again disable the rest (including the one corresponding to the previously enabled channel)
    std::vector<uint8_t> clearAmplitude = {
        0x00  // Amplitude set to zero
    };
    cp2130_.spiWrite(clearAmplitude, EPOUT, errcnt, errstr);  // Set the amplitude to zero (AD5160 on channel 1)
    usleep(100);  // Wait 100us, in order to prevent possible errors while disabling the chip select (workaround)
    cp2130_.disableCS(1, errcnt, errstr);  // Disable the chip select corresponding to channel 1, which is the only one that is active to this point
}

// Closes the device safely, if open
void GF1Device::close()
{
    cp2130_.close();
}

// Returns the silicon version of the CP2130 bridge
CP2130::SiliconVersion GF1Device::getCP2130SiliconVersion(int &errcnt, std::string &errstr)
{
    return cp2130_.getSiliconVersion(errcnt, errstr);
}

// Returns the hardware revision of the device
std::string GF1Device::getHardwareRevision(int &errcnt, std::string &errstr)
{
    return hardwareRevision(getUSBConfig(errcnt, errstr));
}

// Gets the manufacturer descriptor from the device
std::u16string GF1Device::getManufacturerDesc(int &errcnt, std::string &errstr)
{
    return cp2130_.getManufacturerDesc(errcnt, errstr);
}

// Gets the product descriptor from the device
std::u16string GF1Device::getProductDesc(int &errcnt, std::string &errstr)
{
    return cp2130_.getProductDesc(errcnt, errstr);
}

// Gets the serial descriptor from the device
std::u16string GF1Device::getSerialDesc(int &errcnt, std::string &errstr)
{
    return cp2130_.getSerialDesc(errcnt, errstr);
}

// Gets the USB configuration of the device
CP2130::USBConfig GF1Device::getUSBConfig(int &errcnt, std::string &errstr)
{
    return cp2130_.getUSBConfig(errcnt, errstr);
}

// Opens a device and assigns its handle
int GF1Device::open(const std::string &serial)
{
    return cp2130_.open(VID, PID, serial);
}

// Issues a reset to the CP2130, which in effect resets the entire device
void GF1Device::reset(int &errcnt, std::string &errstr)
{
    cp2130_.reset(errcnt, errstr);
}

// Sets the amplitude of the generated signal to the given value (in Vpp)
void GF1Device::setAmplitude(float amplitude, int &errcnt, std::string &errstr)
{
    if (amplitude < AMPLITUDE_MIN || amplitude > AMPLITUDE_MAX) {
        ++errcnt;
        errstr += "In setAmplitude(): Amplitude must be between 0 and 5.\n";  // Program logic error
    } else {
        cp2130_.selectCS(1, errcnt, errstr);  // Enable the chip select corresponding to channel 1, and disable any others
        uint8_t amplitudeCode = static_cast<uint8_t>(amplitude * AQUANTUM / AMPLITUDE_MAX + 0.5);
        std::vector<uint8_t> setAmplitude = {
            amplitudeCode  // Amplitude
        };
        cp2130_.spiWrite(setAmplitude, EPOUT, errcnt, errstr);  // Set the amplitude of the output signal (AD5160 on channel 1)
        usleep(100);  // Wait 100us, in order to prevent possible errors while disabling the chip select (workaround)
        cp2130_.disableCS(1, errcnt, errstr);  // Disable the previously enabled chip select
    }
}

// Sets the frequency of the generated signal to the given value (in KHz)
void GF1Device::setFrequency(float frequency, int &errcnt, std::string &errstr)
{
    if (frequency < FREQUENCY_MIN || frequency > FREQUENCY_MAX) {
        ++errcnt;
        errstr += "In setFrequency(): Frequency must be between 0 and 25000.\n";  // Program logic error
    } else {
        clearCtrlInterrupt(errcnt, errstr);  // Clear "CTRL" and "INTERRUPT" signals
        toggleInterrupt(errcnt, errstr);  // Toggle "INTERRUPT" signal (this toggle is not really necessary, unless the frequency increments are set to be externally triggered via GPIO.2/CTRL)
        cp2130_.selectCS(0, errcnt, errstr);  // Enable the chip select corresponding to channel 0, and disable any others
        uint32_t frequencyCode = static_cast<uint32_t>(frequency * FQUANTUM / MCLK + 0.5);
        std::vector<uint8_t> setFrequency = {
            0x10, 0x00,                                                      // Zero frequency increments
            0x20, 0x00, 0x30, 0x00,                                          // Delta frequency set to zero
            0x40, 0x00,                                                      // Increment interval set to zero
            static_cast<uint8_t>(FSTARTLSB | (0x0f & frequencyCode >> 8)),   // Start frequency (Fstart LSBs register)
            static_cast<uint8_t>(frequencyCode),
            static_cast<uint8_t>(FSTARTMSB | (0x0f & frequencyCode >> 20)),  // Start frequency (Fstart MSBs register)
            static_cast<uint8_t>(frequencyCode >> 12)
        };
        cp2130_.spiWrite(setFrequency, EPOUT, errcnt, errstr);  // Set the frequency of the output signal by updating the above registers (AD5932 on channel 0)
        usleep(100);  // Wait 100us, in order to prevent possible errors while disabling the chip select (workaround)
        cp2130_.disableCS(0, errcnt, errstr);  // Disable the previously enabled chip select
        toggleCtrl(errcnt, errstr);  // Toggle "CTRL" signal
    }
}

// Sets the waveform of the generated signal to sinusoidal
void GF1Device::setSineWave(int &errcnt, std::string &errstr)
{
    clearCtrlInterrupt(errcnt, errstr);  // Clear "CTRL" and "INTERRUPT" signals
    cp2130_.selectCS(0, errcnt, errstr);  // Enable the chip select corresponding to channel 0, and disable any others
    std::vector<uint8_t> setSineWave = {
        0x0f, 0xdf  // Sinusoidal waveform, automatic increments, MSBOUT pin enabled, SYNCOUT pin enabled, B24 = 1, SYNCSEL = 1
    };
    cp2130_.spiWrite(setSineWave, EPOUT, errcnt, errstr);  // Set the waveform to sinusoidal (AD5932 on channel 0)
    usleep(100);  // Wait 100us, in order to prevent possible errors while disabling the chip select (workaround)
    cp2130_.disableCS(0, errcnt, errstr);  // Disable the previously enabled chip select
    toggleCtrl(errcnt, errstr);  // Toggle "CTRL" signal
}

// Sets the waveform of the generated signal to triangular
void GF1Device::setTriangleWave(int &errcnt, std::string &errstr)
{
    clearCtrlInterrupt(errcnt, errstr);  // Clear "CTRL" and "INTERRUPT" signals
    cp2130_.selectCS(0, errcnt, errstr);  // Enable the chip select corresponding to channel 0, and disable any others
    std::vector<uint8_t> setTriangleWave = {
        0x0d, 0xdf  // Triangular waveform, automatic increments, MSBOUT pin enabled, SYNCOUT pin enabled, B24 = 1, SYNCSEL = 1
    };
    cp2130_.spiWrite(setTriangleWave, EPOUT, errcnt, errstr);  // Set the waveform to triangular (AD5932 on channel 0)
    usleep(100);  // Wait 100us, in order to prevent possible errors while disabling the chip select (workaround)
    cp2130_.disableCS(0, errcnt, errstr);  // Disable the previously enabled chip select
    toggleCtrl(errcnt, errstr);  // Toggle "CTRL" signal
}

// Sets up channel 0 for communication with the AD5932 waveform generator
void GF1Device::setupChannel0(int &errcnt, std::string &errstr)
{
    CP2130::SPIMode mode;
    mode.csmode = CP2130::CSMODEPP;  // Chip select pin mode regarding channel 0 is push-pull
    mode.cfrq = CP2130::CFRQ12M;  // SPI clock frequency set to 12MHz
    mode.cpol = CP2130::CPOL1;  // SPI clock polarity is active low (CPOL = 1)
    mode.cpha = CP2130::CPHA0;  // SPI data is valid on each falling edge (CPHA = 0)
    cp2130_.configureSPIMode(0, mode, errcnt, errstr);  // Configure SPI mode for channel 0, using the above settings
    cp2130_.disableSPIDelays(0, errcnt, errstr);  // Disable all SPI delays for channel 0
}

// Sets up channel 1 for communication with the AD5160 SPI potentiometer
void GF1Device::setupChannel1(int &errcnt, std::string &errstr)
{
    CP2130::SPIMode mode;
    mode.csmode = CP2130::CSMODEPP;  // Chip select pin mode regarding channel 1 is push-pull
    mode.cfrq = CP2130::CFRQ12M;  // SPI clock frequency set to 12MHz
    mode.cpol = CP2130::CPOL0;  // SPI clock polarity is active high (CPOL = 0)
    mode.cpha = CP2130::CPHA0;  // SPI data is valid on each rising edge (CPHA = 0)
    cp2130_.configureSPIMode(1, mode, errcnt, errstr);  // Configure SPI mode for channel 1, using the above settings
    cp2130_.disableSPIDelays(1, errcnt, errstr);  // Disable all SPI delays for channel 1
}

// Starts the signal generation
void GF1Device::start(int &errcnt, std::string &errstr)
{
    clearCtrlInterrupt(errcnt, errstr);  // Clear "CTRL" and "INTERRUPT" signals
    toggleCtrl(errcnt, errstr);  // Toggle "CTRL" signal
}

// Stops the signal generation
void GF1Device::stop(int &errcnt, std::string &errstr)
{
    clearCtrlInterrupt(errcnt, errstr);  // Clear "CTRL" and "INTERRUPT" signals
    toggleInterrupt(errcnt, errstr);  // Toggle "INTERRUPT" signal
}

// Helper function that returns the expected amplitude from a given amplitude value
// Note that the function is only valid for values between "AMPLITUDE_MIN" [0] and "AMPLITUDE_MAX" [5]
float GF1Device::expectedAmplitude(float amplitude)
{
    return std::round(amplitude * AQUANTUM / AMPLITUDE_MAX) * AMPLITUDE_MAX / AQUANTUM;
}

// Helper function that returns the expected frequency from a given frequency value
// Note that the function is only valid for values between "FREQUENCY_MIN" [0] and "FREQUENCY_MAX" [25000]
float GF1Device::expectedFrequency(float frequency)
{
    return std::round(frequency * FQUANTUM / MCLK) * MCLK / FQUANTUM;
}

// Helper function that returns the hardware revision from a given USB configuration
std::string GF1Device::hardwareRevision(const CP2130::USBConfig &config)
{
    std::string revision;
    if (config.majrel > 1 && config.majrel <= 27) {
        revision += static_cast<char>(config.majrel + 'A' - 2);  // Append major revision letter (a major release number value of 2 corresponds to the letter "A" and so on)
    }
    if (config.majrel == 1 || config.minrel != 0) {
        std::ostringstream stream;
        stream << static_cast<int>(config.minrel);
        revision += stream.str();  // Append minor revision number
    }
    return revision;
}

// Helper function to list devices
std::list<std::string> GF1Device::listDevices(int &errcnt, std::string &errstr)
{
    return CP2130::listDevices(VID, PID, errcnt, errstr);
}
