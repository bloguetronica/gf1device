/* GF1 device class - Version 1.0.1
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


#ifndef GF1DEVICE_H
#define GF1DEVICE_H

// Includes
#include <cstdint>
#include <list>
#include <string>
#include "cp2130.h"

class GF1Device
{
private:
    CP2130 cp2130_;

    void clearCtrlInterrupt(int &errcnt, std::string &errstr);
    void toggleCtrl(int &errcnt, std::string &errstr);
    void toggleInterrupt(int &errcnt, std::string &errstr);

public:
    // Class definitions
    static const uint16_t VID = 0x10c4;                          // USB vendor ID
    static const uint16_t PID = 0x8a7d;                          // USB product ID
    static const int SUCCESS = CP2130::SUCCESS;                  // Returned by open() if successful
    static const int ERROR_INIT = CP2130::ERROR_INIT;            // Returned by open() in case of a libusb initialization failure
    static const int ERROR_NOT_FOUND = CP2130::ERROR_NOT_FOUND;  // Returned by open() if the device was not found
    static const int ERROR_BUSY = CP2130::ERROR_BUSY;            // Returned by open() if the device is already in use

    // Limits applicable to setAmplitude()
    static constexpr float AMPLITUDE_MIN = 0;  // Minimum amplitude
    static constexpr float AMPLITUDE_MAX = 5;  // Maximum amplitude

    // Limits applicable to setFrequency()
    static constexpr float FREQUENCY_MIN = 0;      // Minimum frequency
    static constexpr float FREQUENCY_MAX = 25000;  // Maximum frequency

    GF1Device();

    bool disconnected() const;
    bool isOpen() const;

    void clear(int &errcnt, std::string &errstr);
    void close();
    CP2130::SiliconVersion getCP2130SiliconVersion(int &errcnt, std::string &errstr);
    std::string getHardwareRevision(int &errcnt, std::string &errstr);
    std::u16string getManufacturerDesc(int &errcnt, std::string &errstr);
    std::u16string getProductDesc(int &errcnt, std::string &errstr);
    std::u16string getSerialDesc(int &errcnt, std::string &errstr);
    CP2130::USBConfig getUSBConfig(int &errcnt, std::string &errstr);
    int open(const std::string &serial = std::string());
    void reset(int &errcnt, std::string &errstr);
    void setAmplitude(float amplitude, int &errcnt, std::string &errstr);
    void setFrequency(float frequency, int &errcnt, std::string &errstr);
    void setSineWave(int &errcnt, std::string &errstr);
    void setTriangleWave(int &errcnt, std::string &errstr);
    void setupChannel0(int &errcnt, std::string &errstr);
    void setupChannel1(int &errcnt, std::string &errstr);
    void start(int &errcnt, std::string &errstr);
    void stop(int &errcnt, std::string &errstr);

    static float expectedAmplitude(float amplitude);
    static float expectedFrequency(float frequency);
    static std::string hardwareRevision(const CP2130::USBConfig &config);
    static std::list<std::string> listDevices(int &errcnt, std::string &errstr);
};

#endif  // GF1DEVICE_H
