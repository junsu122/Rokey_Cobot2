#!/usr/bin/env python3

from pymodbus.client.sync import ModbusTcpClient as ModbusClient


class RG:

    def __init__(self, gripper, ip, port):
        self.client = ModbusClient(
            ip, port=port, stopbits=1, bytesize=8, parity="E", baudrate=115200, timeout=1
        )
        if gripper not in ["rg2", "rg6"]:
            print("Please specify either rg2 or rg6.")
            return
        self.gripper = gripper  # RG2/6
        if self.gripper == "rg2":
            self.max_width = 1100
            self.max_force = 400
        elif self.gripper == "rg6":
            self.max_width = 1600
            self.max_force = 1200
        self.open_connection()

    def open_connection(self):
        """Opens the connection with a gripper."""
        self.client.connect()

    def close_connection(self):
        """Closes the connection with the gripper."""
        self.client.close()

    def get_fingertip_offset(self):
        """Reads the current fingertip offset in 1/10 millimeters."""
        result = self.client.read_holding_registers(address=258, count=1, unit=65)
        return result.registers[0] / 10.0

    def get_width(self):
        """Reads current width between gripper fingers in 1/10 millimeters."""
        result = self.client.read_holding_registers(address=267, count=1, unit=65)
        return result.registers[0] / 10.0

    def get_status(self):
        """Reads current device status (7-bit flags).

        Bit 0: busy          — motion ongoing, not accepting new commands
        Bit 1: grip detected — internal/external grip detected
        Bit 2: S1 pushed
        Bit 3: S1 trigged    — safety circuit 1 activated, power-cycle to reset
        Bit 4: S2 pushed
        Bit 5: S2 trigged    — safety circuit 2 activated, power-cycle to reset
        Bit 6: safety error  — safety switch pushed at power-on
        """
        result = self.client.read_holding_registers(address=268, count=1, unit=65)
        status = format(result.registers[0], "016b")
        messages = [
            "A motion is ongoing so new commands are not accepted.",
            "An internal- or external grip is detected.",
            "Safety switch 1 is pushed.",
            "Safety circuit 1 is activated so it will not move.",
            "Safety switch 2 is pushed.",
            "Safety circuit 2 is activated so it will not move.",
            "Any of the safety switch is pushed.",
        ]
        status_list = [0] * 7
        for i, msg in enumerate(messages):
            if int(status[-(i + 1)]):
                print(msg)
                status_list[i] = 1
        return status_list

    def get_width_with_offset(self):
        """Reads current width between gripper fingers including fingertip offset."""
        result = self.client.read_holding_registers(address=275, count=1, unit=65)
        return result.registers[0] / 10.0

    def set_control_mode(self, command):
        """Sets gripper control mode. Valid values: 1=grip, 8=stop, 16=grip_w_offset."""
        self.client.write_register(address=2, value=command, unit=65)

    def set_target_force(self, force_val):
        """Sets target gripping force in 1/10 N (0–400 for RG2, 0–1200 for RG6)."""
        self.client.write_register(address=0, value=force_val, unit=65)

    def set_target_width(self, width_val):
        """Sets target finger width in 1/10 mm (0–1100 for RG2, 0–1600 for RG6)."""
        self.client.write_register(address=1, value=width_val, unit=65)

    def close_gripper(self, force_val=400):
        """Closes gripper."""
        print("Start closing gripper.")
        self.client.write_registers(address=0, values=[force_val, 0, 16], unit=65)

    def open_gripper(self, force_val=400):
        """Opens gripper."""
        print("Start opening gripper.")
        self.client.write_registers(address=0, values=[force_val, self.max_width, 16], unit=65)

    def move_gripper(self, width_val, force_val=400):
        """Moves gripper to the specified width (1/10 mm)."""
        print("Start moving gripper.")
        self.client.write_registers(address=0, values=[force_val, width_val, 16], unit=65)
