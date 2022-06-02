/* GF1 device class - Version 0.2.1
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
        std::vector<uint8_t> set = {
            amplitudeCode  // Amplitude
        };
        cp2130_.spiWrite(set, EPOUT, errcnt, errstr);  // Set the output voltage by updating the above registers
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
        cp2130_.setGPIO2(false, errcnt, errstr);  // Make sure that both GPIO.2
        cp2130_.setGPIO3(false, errcnt, errstr);  // and GPIO.3 are set to to a logical low first
        cp2130_.setGPIO3(true, errcnt, errstr);  // Then set GPIO.3 to a logical high
        cp2130_.setGPIO3(false, errcnt, errstr);  // and again to a logical low (this toggle is not really necessary, unless the frequency increments are set to be externally triggered via GPIO.2/CTRL)
        cp2130_.selectCS(0, errcnt, errstr);  // Enable the chip select corresponding to channel 0, and disable any others
        uint32_t frequencyCode = static_cast<uint32_t>(frequency * FQUANTUM / MCLK + 0.5);
        std::vector<uint8_t> set = {
            0x10, 0x00,                                                      // Zero frequency increments
            0x20, 0x00, 0x30, 0x00,                                          // Delta frequency set to zero
            0x40, 0x00,                                                      // Increment interval set to zero
            static_cast<uint8_t>(FSTARTLSB | (0x0f & frequencyCode >> 8)),   // Start frequency (Fstart LSBs register)
            static_cast<uint8_t>(frequencyCode),
            static_cast<uint8_t>(FSTARTMSB | (0x0f & frequencyCode >> 20)),  // Start frequency (Fstart MSBs register)
            static_cast<uint8_t>(frequencyCode >> 12)
        };
        cp2130_.spiWrite(set, EPOUT, errcnt, errstr);  // Set the output voltage by updating the above registers
        usleep(100);  // Wait 100us, in order to prevent possible errors while disabling the chip select (workaround)
        cp2130_.disableCS(0, errcnt, errstr);  // Disable the previously enabled chip select
        cp2130_.setGPIO2(true, errcnt, errstr);  // Set GPIO.2 to a logical high
        cp2130_.setGPIO2(false, errcnt, errstr);  // and then to a logical low
    }
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
