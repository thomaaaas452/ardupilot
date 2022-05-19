/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
   OpenMV library
*/

#define AP_SERIALMANAGER_OPEN_MV_BAUD         115200
#define AP_SERIALMANAGER_OPENMV_BUFSIZE_RX        64
#define AP_SERIALMANAGER_OPENMV_BUFSIZE_TX        64

#include "AP_K210.h"

extern const AP_HAL::HAL& hal;

//constructor
AP_K210::AP_K210(void)
{
    _port = NULL;
    _step = 0;
}

// init - perform require initialisation including detecting which protocol to use
void AP_K210::init(const AP_SerialManager& serial_manager)
{
    // check for DEVO_DPort
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_K210, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        _port->begin(AP_SERIALMANAGER_OPEN_MV_BAUD, AP_SERIALMANAGER_OPENMV_BUFSIZE_RX, AP_SERIALMANAGER_OPENMV_BUFSIZE_TX);
    }
}

void AP_K210::update()
{
    if(_port == NULL)
        return;

    int16_t numc = _port->available();
    uint8_t data;
    uint8_t checksum = 0;

    for (int16_t i = 0; i < numc; i++) {
        data = _port->read();

        switch(_step) {
        case 0:
            if(data == 0xA5)
                _step = 1;
            break;

        case 1:
            if(data == 0x5A)
                _step = 2;
            else
                _step = 0;
            break;

        case 2:
            _cx_temp = data;
            _step = 3;
            break;

        case 3:
            _cy_temp = data;
            _step = 4;
            break;

        case 4:
            _cz_high_temp = data;
            _step = 5;
        
        case 5:
            _cz_low_temp = data;
            _step = 6;

        case 6:
            checksum = _cx_temp + _cy_temp ;
            if(checksum == data) {
                cx = _cx_temp*6;
                cy = _cy_temp*6;
                cz = _cz_high_temp*256+_cz_low_temp;
                last_frame_ms = AP_HAL::millis();
            }

            _step = 0;
            break;

        default:
            _step = 0;
        }
    }
}