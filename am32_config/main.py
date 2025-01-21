#!/usr/bin/env python3
import sys
import os
import time
import argparse
from am32_config import AM32_CONFIG, FOUR_WAY_COMMANDS

def print_success(text):
    print(f"\033[92m{text}\033[0m")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='AM32 Config Tool')
    parser.add_argument('-p', '--port', required=True, help='Serial port to use')
    parser.add_argument('-b', '--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('-d', '--debug', action='store_true', help='Enable debug mode')
    args = parser.parse_args()

    am32_config = AM32_CONFIG(args.port, baudrate=args.baud, debug=args.debug)

    if args.debug:
        print("Debug mode enabled")
    
    print("initializing...")
    time.sleep(2)
    print_success("Connected to device")
    
    # Send and receive MSP commands
    print("Sending MSP_API_VERSION...")
    response = am32_config.send_msp_api_version()
    if not response:
        print("No response for MSP_API_VERSION")
        am32_config.close()
        sys.exit(1)
    parsed_response = am32_config.process_response(response)
    print_success(f"Got msp data: {am32_config.decode_response(parsed_response)}")
    
    print("Sending MSP_FC_VARIANT...")
    response = am32_config.send_msp_fc_variant()
    if not response:
        print("No response for MSP_FC_VARIANT")
        am32_config.close()
        sys.exit(1)
    parsed_response = am32_config.process_response(response)
    print_success(f"Got msp data: {am32_config.decode_response(parsed_response)}")
    
    print("Sending MSP_BATTERY_STATE...")
    response = am32_config.send_msp_battery_state()
    if not response:
        print("No response for MSP_BATTERY_STATE")
        am32_config.close()
        sys.exit(1)
    parsed_response = am32_config.process_response(response) # type: ignore
    print_success(f"Got msp data: {am32_config.decode_response(parsed_response)}")
    
    print("Sending MSP_MOTOR...")
    response = am32_config.send_msp_motor()
    if not response:
        print("No response for MSP_MOTOR")
        am32_config.close()
        sys.exit(1)
    parsed_response = am32_config.process_response(response)
    print_success(f"Got msp data: {am32_config.decode_response(parsed_response)}")

    print("Sending MSP_MOTOR_CONFIG...")
    response = am32_config.send_msp_motor_config()
    if not response:
        print("No response for MSP_MOTOR_CONFIG")
        am32_config.close()
        sys.exit(1)
    parsed_response = am32_config.process_response(response)
    print_success(f"Got msp data: { am32_config.decode_response(parsed_response)}")
    
    print("Sending MSP_SET_PASSTHROUGH...")
    response = am32_config.send_msp_set_passthrough()
    if not response:
        print("No response for MSP_SET_PASSTHROUGH")
        am32_config.close()
        sys.exit(1)
    parsed_response = am32_config.process_response(response)
    print_success(f"Got msp data: { am32_config.decode_response(parsed_response)}")

    time.sleep(1) # Wait for the passthrough to be enabled

    SKIP_ESC_INDEX = [1, 2, 3] 
    firmware_path = os.path.join(os.path.dirname(__file__), "..", "AM32_VIMDRONES_L431_2.17.bin")
    # Send and receive FOUR_WAY commands
    for esc_index in range(am32_config.get_motor_count()):
        if esc_index in SKIP_ESC_INDEX:
            continue
        esc_params =  am32_config.read_eeprom_params_from_single_esc(esc_index)
        if esc_params is None:
            # Empty EEPROM params, write default params and read again
            print(f"ESC {esc_index + 1}: Empty EEPROM params, writing default params")
            default_eeprom_path = os.path.join(os.path.dirname(__file__), '..', 'eeprom_default.bin') 
            am32_config.reset_default_params(esc_index, default_eeprom_path)
            esc_params =  am32_config.read_eeprom_params_from_single_esc(esc_index)

        if esc_params:
            # print(f"ESC {esc_index + 1}: eeprom_params:", eeprom_params[esc_index])
            print_success(f"#ESC {esc_index + 1}: Firmware Name: {esc_params.get('firmware_name')}")
            print_success(f"#ESC {esc_index + 1}: Firmware Version: {esc_params.get('version_major')}.{esc_params.get('version_minor')}")
            if am32_config.flash_esc_firmware(esc_index, firmware_path):
                print_success(f"#ESC {esc_index + 1}: Successfully flashed firmware")
        
        else :
            print(f"Failed to read EEPROM params for ESC {esc_index + 1}")
    
    # Close the serial port
    am32_config.close()