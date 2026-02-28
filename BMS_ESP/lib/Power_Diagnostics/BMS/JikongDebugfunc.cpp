#include "Jikong_Handler.h"


void JikongMessenger::print_all_data(Stream& out) {
    out.println(F("==== Jikong BMS Data Dump ===="));

    // ---- Temperatures ----
    out.print(F("Power tube temp (dC): "));
    out.println(get_power_tube_temp_dC());

    out.print(F("Battery temp (dC): "));
    out.println(get_battery_temp_dC());

    out.print(F("Battery box temp (dC): "));
    out.println(get_battery_box_temp_dC());

    // ---- Electrical ----
    out.print(F("Total voltage (mV): "));
    out.println(get_total_voltage_mV());

    out.print(F("Current (dA): "));
    out.println(get_current_dA());

    out.print(F("Remaining capacity (%): "));
    out.println(get_remaining_capacity_pct());

    out.print(F("Temp sensor count: "));
    out.println(get_temp_sensor_count_n());

    out.print(F("Cycle count: "));
    out.println(get_cycle_count());

    out.print(F("Total cycle capacity (Ah): "));
    out.println((unsigned long)get_total_cycle_capacity_Ah());

    out.print(F("String count: "));
    out.println(get_string_count_n());

    // ---- Voltage protections ----
    out.print(F("Pack overvoltage (mV): "));
    out.println(get_pack_overvoltage_mV());

    out.print(F("Pack undervoltage (mV): "));
    out.println(get_pack_undervoltage_mV());

    out.print(F("Cell overvoltage (mV): "));
    out.println(get_cell_overvoltage_mV());

    out.print(F("Cell undervoltage (mV): "));
    out.println(get_cell_undervoltage_mV());

    // ---- Balancing ----
    out.print(F("Balance start voltage (mV): "));
    out.println(get_balance_start_voltage_mV());

    out.print(F("Balance diff (mV): "));
    out.println(get_balance_diff_mV());

    out.print(F("Active balancing: "));
    out.println(is_active_balance_enabled() ? F("YES") : F("NO"));

    // ---- MOS state ----
    out.print(F("Charge MOS: "));
    out.println(is_charge_mos_enabled() ? F("ON") : F("OFF"));

    out.print(F("Discharge MOS: "));
    out.println(is_discharge_mos_enabled() ? F("ON") : F("OFF"));

    // ---- Strings ----
    out.print(F("Battery type: "));
    out.println(get_battery_type() ?: "N/A");

    out.print(F("Firmware: "));
    out.println(get_firmware_version() ?: "N/A");
    out.printf("Manufacturer ID: %u\n", get_manufacturer_id() );

    out.println(F("==== End Dump ===="));
}

void JikongMessenger::testFunction1(byte* testArr){
    Dynamic_Array _msg;
    testFunction2(_msg, testArr);
    for(int i = 0; i < _msg.index; i++){
        Serial.println(_msg.body[i], HEX);
    }
    _parseData(_msg.body, _msg.index);
    free(_msg.body);
    _msg.body = nullptr;
}

void JikongMessenger::testFunction2(Dynamic_Array &msg, byte* testArr){
 
    msg.index = 285; 
    uint8_t buffer[520];
    
    memcpy(buffer, testArr, 285);
    
    msg.body = (byte*)malloc(msg.index);
 
        if (!msg.body) {
        Serial.println("Memory allocation failed!");
        return;
    }
    memcpy(msg.body, &buffer, msg.index);
}   

void JikongMessenger::printfromstruct(){
    #define PRINT_BOOL(label, val) \
    Serial.printf("%s: %s\n", label, (val) ? "true" : "false");

    #define PRINT_STR(label, val) \
    Serial.printf("%s: %s\n", label, (val) ? (val) : "(null)");

    Serial.println("\n========== STORED DATA DUMP ==========\n");

    //--------------- Cell Voltages ---------------- 
    Serial.println("Cell Voltages (mV):");
    if (_Stored_Data.cell_voltage_mV && _Stored_Data.string_count_n > 0) {
        for (uint16_t i = 0; i < _Stored_Data.string_count_n; i++) {
            Serial.printf("  Cell %u: %u mV\n", i + 1, _Stored_Data.cell_voltage_mV[i].cellVoltage);
        }
    } else {
        Serial.println("  (none)");
    }

    // ---------------- Temperatures ---------------- //
    Serial.printf("Power Tube Temp: %d dC\n", _Stored_Data.power_tube_temp_dC);
    Serial.printf("Battery Box Temp: %d dC\n", _Stored_Data.battery_box_temp_dC);
    Serial.printf("Battery Temp: %d dC\n", _Stored_Data.battery_temp_dC);

    // ---------------- Electrical ---------------- //
    Serial.printf("Total Voltage: %u mV\n", _Stored_Data.total_voltage_mV);
    Serial.printf("Current: %d dA\n", _Stored_Data.current_dA);
    Serial.printf("Remaining Capacity: %u %%\n", _Stored_Data.remaining_capacity_pct);
    Serial.printf("Temp Sensor Count: %u\n", _Stored_Data.temp_sensor_count_n);
    Serial.printf("Cycle Count: %u\n", _Stored_Data.cycle_count);
    Serial.printf("Total Cycle Capacity: %u Ah\n", _Stored_Data.total_cycle_capacity_Ah);
    Serial.printf("String Count: %u\n", _Stored_Data.string_count_n);

    // ---------------- Voltage Protections ---------------- //
    Serial.printf("Pack OV: %u mV\n", _Stored_Data.pack_overvoltage_mV);
    Serial.printf("Pack UV: %u mV\n", _Stored_Data.pack_undervoltage_mV);

    Serial.printf("Cell OV: %u mV\n", _Stored_Data.cell_overvoltage_mV);
    Serial.printf("Cell OV Recovery: %u mV\n", _Stored_Data.cell_overvoltage_recovery_mV);
    Serial.printf("Cell OV Delay: %u s\n", _Stored_Data.cell_overvoltage_delay_s);

    Serial.printf("Cell UV: %u mV\n", _Stored_Data.cell_undervoltage_mV);
    Serial.printf("Cell UV Release: %u mV\n", _Stored_Data.Monomer_undervoltage_release_mV);
    Serial.printf("Cell UV Delay: %u s\n", _Stored_Data.cell_undervoltage_delay_s);

    Serial.printf("Cell Pressure Diff: %u mV\n", _Stored_Data.cell_pressure_diff_protection_mV);

    // ---------------- Current Protections ---------------- //
    Serial.printf("Discharge OC: %u A\n", _Stored_Data.discharge_overcurrent_A);
    Serial.printf("Discharge OC Delay: %u s\n", _Stored_Data.discharge_overcurrent_delay_s);

    Serial.printf("Charge OC: %u A\n", _Stored_Data.charge_overcurrent_A);
    Serial.printf("Charge OC Delay: %u s\n", _Stored_Data.charge_overcurrent_delay_s);

    // ---------------- Balancing ---------------- //
    Serial.printf("Balance Start Voltage: %u mV\n", _Stored_Data.balance_start_voltage_mV);
    Serial.printf("Balance Diff: %u mV\n", _Stored_Data.balance_diff_mV);
    PRINT_BOOL("Active Balance Enabled", _Stored_Data.active_balance_enabled);

    // ---------------- Temperature Protections ---------------- //
    Serial.printf("Power Tube Temp Protect: %d dC\n", _Stored_Data.Power_tube_temp_protection_value_dC);
    Serial.printf("Power Tube Temp Recovery: %d dC\n", _Stored_Data.Power_tube_temp_recovery_value_dC);

    Serial.printf("Box High Temp: %d dC\n", _Stored_Data.box_high_temp_dC);
    Serial.printf("Box Temp Recovery: %d dC\n", _Stored_Data.box_temp_recovery_dC);

    Serial.printf("Battery Temp Diff: %d dC\n", _Stored_Data.battery_temp_diff_dC);

    Serial.printf("Charge High Temp: %d dC\n", _Stored_Data.charge_high_temp_dC);
    Serial.printf("Discharge High Temp: %d dC\n", _Stored_Data.discharge_high_temp_dC);

    Serial.printf("Charge Low Temp: %d dC\n", _Stored_Data.charge_low_temp_dC);
    Serial.printf("Charge Low Temp Recovery: %d dC\n", _Stored_Data.charge_low_temp_recovery_dC);

    Serial.printf("Discharge Low Temp: %d dC\n", _Stored_Data.discharge_low_temp_dC);
    Serial.printf("Discharge Low Temp Recovery: %d dC\n", _Stored_Data.discharge_low_temp_recovery_dC);

    // ---------------- Configuration ---------------- //
    Serial.printf("Battery String Setting: %u\n", _Stored_Data.battery_string_setting);
    Serial.printf("Total Capacity Setting: %u Ah\n", _Stored_Data.battery_capacity_setting);
    Serial.printf("Battery Capacity: %u Ah\n", _Stored_Data.battery_capacity_Ah);

    PRINT_BOOL("Charge MOS Enabled", _Stored_Data.charge_mos_enabled);
    PRINT_BOOL("Discharge MOS Enabled", _Stored_Data.discharge_mos_enabled);

    Serial.printf("Current Calibration Offset: %d\n", _Stored_Data.current_calibration_offset);
    Serial.printf("P Board Address: %u\n", _Stored_Data.P_board_address);
    PRINT_STR("Battery Type Code: ", _Stored_Data.battery_type);
    Serial.printf("Sleep Delay: %u s\n", _Stored_Data.sleep_delay_s);
    Serial.printf("Low Capacity Alarm: %u %%\n", _Stored_Data.low_cap_alarmVol_pct);

    PRINT_STR("Parameter Password", _Stored_Data.parameter_password);
    PRINT_BOOL("Dedicated Charger Enabled", _Stored_Data.dedicated_charger_enabled);

    Serial.printf("Device ID: %llu\n", _Stored_Data.device_id);
 
    Serial.printf("Manufacture Year %u \n", _Stored_Data.manufacture_year);

    Serial.printf("Manufacture Month %u \n", _Stored_Data.manufacture_month);

    Serial.printf("Runtime Hours: %u\n", _Stored_Data.runtime_hours);
    PRINT_STR("Firmware Version", _Stored_Data.firmware_version);

    PRINT_BOOL("Current Calibration Enabled", _Stored_Data.current_calibration_enabled);

    Serial.printf("Actual Capacity: %u Ah\n", _Stored_Data.actual_capacity_Ah);
    Serial.printf("Manufacturer ID: %u\n", _Stored_Data.manufacturer_id);
    Serial.printf("Protocol Version: %s\n", _Stored_Data.protocol_version_number);

    Serial.println("\n======================================\n");
}