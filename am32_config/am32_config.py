import serial
import time
import sys

EEPROM_OFFSET = 0xF800
EEPROM_LENGTH = 184

EEPROM_STRUCT = {
    'reserved_0': 0,
    'eeprom_version': 1,
    'reserved_1': 2,
    'version_major': 3,
    'version_minor': 4,
    'firmware_name': 5,
    'dir_reversed': 17,
    'bi_direction': 18,
    'use_sine_start': 19,
    'comp_pwm': 20,
    'variable_pwm': 21,
    'stuck_rotor_protection': 22,
    'advance_level': 23,
    'pwm_frequency': 24,
    'startup_power': 25,
    'motor_kv': 26,
    'motor_poles': 27,
    'brake_on_stop': 28,
    'stall_protection': 29,
    'beep_volume': 30,
    'telementry_on_interval': 31,
    'servo_low_threshold': 32,
    'servo_high_threshold': 33,
    'servo_neutral': 34,
    'servo_dead_band': 35,
    'low_voltage_cut_off': 36,
    'low_cell_volt_cutoff': 37,
    'rc_car_reverse': 38,
    'use_hall_sensors': 39,
    'sine_mode_changeover_thottle_level': 40,
    'drag_brake_strength': 41,
    'driving_brake_strength': 42,
    'limits_temperature': 43,
    'limits_current': 44,
    'sine_mode_power': 45,
    'input_type': 46,
    'auto_advance': 47,
    'tune': 48,
    'can_node': 176,
    'esc_index': 177,
    'require_arming': 178,
    'telem_rate': 179,
    'require_zero_throttle': 180,
    'filter_hz': 181,
    'debug_rate': 182,
    'term_enable': 183,
    'can_reserved': 184
}

# FOUR_WAY_COMMANDS enum
FOUR_WAY_COMMANDS = {
    'cmd_InterfaceTestAlive': 0x30,
    'cmd_ProtocolGetVersion': 0x31,
    'cmd_InterfaceGetName': 0x32,
    'cmd_InterfaceGetVersion': 0x33,
    'cmd_InterfaceExit': 0x34,
    'cmd_DeviceReset': 0x35,
    'cmd_DeviceInitFlash': 0x37,
    'cmd_DeviceEraseAll': 0x38,
    'cmd_DevicePageErase': 0x39,
    'cmd_DeviceRead': 0x3A,
    'cmd_DeviceWrite': 0x3B,
    'cmd_DeviceC2CK_LOW': 0x3C,
    'cmd_DeviceReadEEprom': 0x3D,
    'cmd_DeviceWriteEEprom': 0x3E,
    'cmd_InterfaceSetMode': 0x3F,
}

# FOUR_WAY_ACK enum
FOUR_WAY_ACK = {
    'ACK_OK': 0x00,
    'ACK_I_UNKNOWN_ERROR': 0x01,
    'ACK_I_INVALID_CMD': 0x02,
    'ACK_I_INVALID_CRC': 0x03,
    'ACK_I_VERIFY_ERROR': 0x04,
    'ACK_D_INVALID_COMMAND': 0x05,
    'ACK_D_COMMAND_FAILED': 0x06,
    'ACK_D_UNKNOWN_ERROR': 0x07,
    'ACK_I_INVALID_CHANNEL': 0x08,
    'ACK_I_INVALID_PARAM': 0x09,
    'ACK_D_GENERAL_ERROR': 0x0F,
}

class AM32_CONFIG:
    MSP_HEADER = bytearray([0x24, 0x4D, 0x3C])
    MSP_API_VERSION = 0x01
    MSP_FC_VARIANT = 0x02
    MSP_FC_VERSION = 0x03
    MSP_BOARD_INFO = 0x04
    MSP_BUILD_INFO = 0x05
    MSP_FEATURE_CONFIG = 0x24
    MSP_MOTOR_3D_CONFIG = 0x7C
    MSP_BATTERY_STATE = 0x82
    MSP_SET_MOTOR = 0xD6
    MSP_SET_PASSTHROUGH = 0xF5
    MSP_IDENT = 0x64
    MSP_STATUS = 0x65
    MSP_MOTOR = 0x68
    MSP_MOTOR_CONFIG = 0x83
    MSP_SET_3D = 0xD9
    MSP_UID = 0xA0
    MSP2_SEND_DSHOT_COMMAND = 0x3003

    def __init__(self, port, baudrate=115200, timeout=1, debug=False):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.fc_response = {}
        self.debug = debug

    def reset():
        self.fc_response = {}

    def send_msp_command(self, command, wait_time=0.5):
        self.ser.write(command)
        if self.debug:
            print(f"Sent command (hex): {' '.join(format(x, '02x') for x in command)}")
        time.sleep(wait_time)  # Increase wait time for the response
        response = self.ser.read(self.ser.in_waiting or 64)  # Read more bytes
        response_hex = ' '.join(format(x, '02x') for x in response)
        if self.debug:
            print(f"Raw response (hex): {response_hex}")
        return response

    def process_response(self, data):
        state = 0
        message_buffer = bytearray()
        message_checksum = 0
        message_length_expected = 0
        message_length_received = 0
        command = None

        for char in data:
            if state == 0:
                if char == ord('$'):
                    state += 1
            elif state == 1:
                if char == ord('M'):
                    state += 1
                else:
                    state = 0
            elif state == 2:
                if char in [ord('<'), ord('>'), ord('!')]:
                    state += 1
                else:
                    state = 0
                    print(f"Unknown msp command direction '{char}'")
            elif state == 3:
                message_length_expected = char
                message_checksum = char
                message_buffer = bytearray(message_length_expected)
                state += 1
            elif state == 4:
                command = char
                message_checksum ^= char
                if message_length_expected > 0:
                    state += 1
                else:
                    state += 2
            elif state == 5:
                message_buffer[message_length_received] = char
                message_checksum ^= char
                message_length_received += 1
                if message_length_received >= message_length_expected:
                    state += 1
            elif state == 6:
                if message_checksum == char:
                    return {
                        'command': command,
                        'data': message_buffer
                    }
                else:
                    print("Checksum mismatch")
                    return None
        return None

    def decode_response(self, parsed_response):
        if not parsed_response:
            return "Invalid or incomplete response"
        
        command = parsed_response['command']
        data = parsed_response['data']
        
        if command == self.MSP_API_VERSION:  # MSP_API_VERSION
            if len(data) < 3:
                return "Incomplete API_VERSION data"
            self.fc_response['protocol_version'] = data[0]
            self.fc_response['api_version'] = f"{data[1]}.{data[2]}.0"
            return f"API_VERSION: {self.fc_response.get('api_version')}"
        elif command == self.MSP_FC_VARIANT:  # MSP_FC_VARIANT
            variant = data.decode('ascii')
            if variant == 'BTFL':
                self.fc_response['fc_variant'] = 'BETAFLIGHT'
            elif variant == "KISS":
                self.fc_response['fc_variant'] = 'KISS'
            elif variant == "INAV":
                self.fc_response['fc_variant'] = 'INAV'
            elif variant == "ARDU":
                self.fc_response['fc_variant'] = 'ARDUPILOT'
            else:
                self.fc_response['fc_variant'] = 'UNKNOWN'
            return f"FC_VARIANT: {self.fc_response.get('fc_variant')}"
        elif command == self.MSP_BATTERY_STATE:  # MSP_BATTERY_STATE
            if len(data) < 8:
                return "Incomplete BATTERY_STATE data"
            self.fc_response['battery_cell_count'] = data[0]
            self.fc_response['battery_capacity'] = int.from_bytes(data[1:3], 'little')  # mAh
            self.fc_response['battery_voltage'] = data[3] / 10.0  # V
            self.fc_response['battery_drawn'] = int.from_bytes(data[4:6], 'little')  # mAh
            self.fc_response['battery_amps'] = int.from_bytes(data[6:8], 'little') / 100.0  # A
            return (f"BATTERY INFO: Cell Count: {self.fc_response.get('battery_cell_count')}, "
                    f"Capacity: {self.fc_response.get('battery_capacity')} mAh, "
                    f"Voltage: {self.fc_response.get('battery_voltage')} V, "
                    f"Drawn: {self.fc_response.get('battery_drawn')} mAh, "
                    f"Amps: {self.fc_response.get('battery_amps')} A")
        elif command == self.MSP_MOTOR:  # MSP_MOTOR
            motor_count = 0
            for i in range(0, len(data), 2):
                if int.from_bytes(data[i:i+2], 'little') > 0:
                    motor_count += 1
            self.fc_response['motor_count'] = motor_count
            return f"MOTOR COUNT: {self.fc_response.get('motor_count')}"
        elif command == self.MSP_MOTOR_CONFIG:  # MSP_MOTOR_CONFIG
            if len(data) < 7:
                return "Incomplete MOTOR_CONFIG data"
            self.fc_response['motor_count'] = data[6]
            return f"MOTOR COUNT: {self.fc_response.get('motor_count')}"
        elif command == self.MSP_SET_PASSTHROUGH:  # MSP_SET_PASSTHROUGH
            if len(data) < 1:
                return "Incomplete SET_PASSTHROUGH data"
            passthrough_status = data[0]
            return f"SET_PASSTHROUGH STATUS: {passthrough_status}"
        else:
            return f"Unknown command {command} with data: {data}"

    def send_four_way_command(self, command, params, address=0, retries=3, wait_time=0.5):
        for attempt in range(retries):
            if len(params) == 0:
                params.append(0)
            elif len(params) > 256:
                print('Too many parameters', len(params))
                return None

            buffer_out = bytearray(7 + len(params))
            buffer_out[0] = 0x2F
            buffer_out[1] = command
            buffer_out[2] = (address >> 8) & 0xFF
            buffer_out[3] = address & 0xFF
            buffer_out[4] = len(params) if len(params) < 256 else 0

            buffer_out[5:5+len(params)] = params

            checksum = self.crc16_xmodem(buffer_out[:-2])
            buffer_out[-2] = (checksum >> 8) & 0xFF
            buffer_out[-1] = checksum & 0xFF

            self.ser.write(buffer_out)
            time.sleep(wait_time)
            response = self.ser.read(self.ser.in_waiting or 64)
            response_hex = ' '.join(format(x, '02x') for x in response)
            if self.debug:
                print(f"Raw response (hex): {response_hex}")

            parsed_response = self.process_four_way_response(response)
            if parsed_response and parsed_response['ack'] == FOUR_WAY_ACK['ACK_OK']:
                return parsed_response
            else:
                print(f"error: {parsed_response['ack'] if parsed_response else 'No response'}")
        
        print("max retries reached")
        return None

    def crc16_xmodem(self, data):
        crc = 0
        poly = 0x1021
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ poly
                else:
                    crc <<= 1
        return crc & 0xFFFF

    def process_four_way_response(self, data):
        if len(data) < 9:
            print("Not enough data")
            return None

        param_count = data[4]
        if param_count == 0:
            param_count = 256

        if len(data) < 8 + param_count:
            print("Not enough data")
            return None

        message = {
            'command': data[1],
            'address': (data[2] << 8) | data[3],
            'ack': data[5 + param_count],
            'checksum': (data[6 + param_count] << 8) | data[7 + param_count],
            'params': data[5:5 + param_count]
        }

        checksum = self.crc16_xmodem(data[:6 + param_count])
        if checksum != message['checksum']:
            print(f"Checksum mismatch, received: {message['checksum']}, calculated: {checksum}")
            return None

        return message

    def close(self):
        self.ser.close()

    def send_msp_api_version(self):
        command = self.MSP_HEADER + bytearray([0x00, self.MSP_API_VERSION, 0x00 ^ self.MSP_API_VERSION])
        return self.send_msp_command(command)

    def send_msp_fc_variant(self):
        command = self.MSP_HEADER + bytearray([0x00, self.MSP_FC_VARIANT, 0x00 ^ self.MSP_FC_VARIANT])
        return self.send_msp_command(command)

    def send_msp_battery_state(self):
        command = self.MSP_HEADER + bytearray([0x00, self.MSP_BATTERY_STATE, 0x00 ^ self.MSP_BATTERY_STATE])
        return self.send_msp_command(command)

    def send_msp_motor(self):
        command = self.MSP_HEADER + bytearray([0x00, self.MSP_MOTOR, 0x00 ^ self.MSP_MOTOR])
        return self.send_msp_command(command)

    def send_msp_motor_config(self):
        command = self.MSP_HEADER + bytearray([0x00, self.MSP_MOTOR_CONFIG, 0x00 ^ self.MSP_MOTOR_CONFIG])
        return self.send_msp_command(command)

    def send_msp_set_passthrough(self):
        command = self.MSP_HEADER + bytearray([0x00, self.MSP_SET_PASSTHROUGH, 0x00 ^ self.MSP_SET_PASSTHROUGH])
        return self.send_msp_command(command)
    
    def get_motor_count(self):
        return self.fc_response.get('motor_count', 0)

    def parse_eeprom_data(self, data):
        eeprom_dict = {}
        for key, address in EEPROM_STRUCT.items():
            if key == 'firmware_name':
                eeprom_dict[key] = data[address:address+12].decode('ascii').strip('\x00')
            elif key == 'tune':
                eeprom_dict[key] = list(data[address:address+128])
            elif key == 'can_reserved':
                eeprom_dict[key] = data[address:address+8]
            else:
                eeprom_dict[key] = int(data[address])
        return eeprom_dict

    def get_param_byte_len(self, key):
        if key == 'firmware_name':
            return 12
        elif key == 'tune':
            return 128
        elif key == 'can_reserved':
            return 8
        else:
            return 1
    
    def parse_eeprom_params(self, key, data):
        if key == 'firmware_name':
            return data.decode('ascii').strip('\x00')
        elif key == 'tune':
            return list(data)
        elif key == 'can_reserved':
            return data
        else:
            return int(data[0])
        
    def read_mcu_info(self):
        pass

    def read_eeprom_params_from_device(self):
        eeprom_params = {}
        targets = []
        for esc_index in range(self.get_motor_count()):
            eeprom_params[esc_index] = {}
            targets.append(EEPROM_LENGTH)

        for esc_index in range(self.get_motor_count()):
            response = self.send_four_way_command(FOUR_WAY_COMMANDS['cmd_DeviceRead'], targets, EEPROM_OFFSET)
            if not response:
                print(f"No response for cmd_DeviceRead at address {EEPROM_OFFSET}")
                return False
            eeprom_params[esc_index] = self.parse_eeprom_data(response['params'])
        return eeprom_params