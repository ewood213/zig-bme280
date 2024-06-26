// Includes
const std = @import("std");

// Constants
pub const ADDR_LOW: u8 = 0x76;
pub const ADDR_HIGH: u8 = 0x77;

// Registers
const ID_REG = 0xD0;
const ID_VAL = 0x60;
const CTRL_MEAS_REG = 0xF4;
const CTRL_HUM_REG = 0xF2;
const CONFIG_REG = 0xF5;
const STATUS_REG = 0xF3;

const RESET_REG = 0xE0;
const RESET_VAL = 0xB6;

const CALIB1_REG_START = 0x88;
const CALIB1_REG_END = 0xA2;

const CALIB2_REG_START = 0xE1;
const CALIB2_REG_END = 0xE8;

const PRESS_MSB_REG = 0xF7;
const PRESS_LSB_REG = 0xF8;
const PRESS_XLSB_REG = 0xF9;

const TEMP_MSB_REG = 0xFA;
const TEMP_LSB_REG = 0xFB;
const TEMP_XLSB_REG = 0xFC;

const HUM_MSB_REG = 0xFD;
const HUM_LSB_REG = 0xFE;

// Enums
pub const SensorMode = enum { Sleep, Forced, Normal };
pub const SensorType = enum { Humidity, Pressure, Temperature };
pub const OverSamplingVal = enum { Zero, One, Two, Four, Eight, Sixteen };
pub const StandbyTimeVal = enum { HalfMs, TenMs, TwentyMs, SixtyTwoFiveMs, OneTwoFiveMs, TwoFiveZeroMs, FiveHundredMs, OneSecond };
pub const FilterCoef = enum { Off, Two, Four, Eight, Sixteen };

// Errors
pub const SensorError = error{ProbeError};

// Types
pub const Calibration = struct {
    pub const Temperature = struct {
        dig_T1: u16,
        dig_T2: i16,
        dig_T3: i16,
    };

    pub const Pressure = struct {
        dig_P1: u16,
        dig_P2: i16,
        dig_P3: i16,
        dig_P4: i16,
        dig_P5: i16,
        dig_P6: i16,
        dig_P7: i16,
        dig_P8: i16,
        dig_P9: i16,
    };

    pub const Humidity = struct {
        dig_H1: u8,
        dig_H2: i16,
        dig_H3: u8,
        dig_H4: i16,
        dig_H5: i16,
        dig_H6: i16,
    };

    temperature: Temperature,
    pressure: Pressure,
    humidity: Humidity,
};

pub const Data = struct { temperature: f64, pressure: f64, humidity: f64 };

// Concat two bytes together to form i16 or u16
fn byte_concat16(comptime T: type, msb: u8, lsb: u8) T {
    comptime std.debug.assert(T == u16 or T == i16);
    return @shlWithOverflow(@as(T, msb), 8)[0] | lsb;
}

// Creates a bme280 type that is templated over the device interface
// The device interface must have these functions:
// init(addr: u8) void
// deinit() void
// read(reg: u8, len: u32) !(if (len == 1) u8 else [len]u8)()
// write(reg: u8, val: u8) !void
// TODO: assert that these functions exist
pub fn create_bme280_type(comptime dev_intf_type: type) type {
    return struct {
        // Constants
        pub const bme280: type = @This();

        // Fields
        dev_intf: dev_intf_type,
        calibration: Calibration = undefined,

        // Functions
        pub fn init() !bme280 {
            const dev_intf = try dev_intf_type.init();
            const calibration = try query_calibration(dev_intf);
            const sensor = bme280{ .dev_intf = dev_intf, .calibration = calibration };
            if (!try sensor.probe()) {
                return SensorError.ProbeError;
            }
            try sensor.reset();
            return sensor;
        }

        pub fn deinit(self: bme280) void {
            self.dev_intf.deinit();
        }

        pub fn put_mode(self: bme280, mode: SensorMode) !void {
            const ctrl_byte: u8 = switch (mode) {
                .Sleep => 0x00,
                .Forced => 0x01,
                .Normal => 0x03,
            };
            const cur_val = try self.dev_intf.read(CTRL_MEAS_REG, 1);
            const next_val = (cur_val & 0xFC) | ctrl_byte;
            try self.dev_intf.write(CTRL_MEAS_REG, next_val);
        }

        pub fn set_oversampling(self: bme280, sensor_type: SensorType, oversampling: OverSamplingVal) !void {
            const oversample_reg_val: u8 = switch (oversampling) {
                .Zero => 0x00,
                .One => 0x01,
                .Two => 0x02,
                .Four => 0x03,
                .Eight => 0x04,
                .Sixteen => 0x05,
            };

            switch (sensor_type) {
                .Humidity => {
                    const cur_val = try self.dev_intf.read(CTRL_HUM_REG, 1);
                    const next_val = (cur_val & 0xF8) | oversample_reg_val;
                    try self.dev_intf.write(CTRL_HUM_REG, next_val);
                },
                .Pressure => {
                    const cur_val = try self.dev_intf.read(CTRL_MEAS_REG, 1);
                    const next_val = (cur_val & 0x8F) | (oversample_reg_val << 5);
                    try self.dev_intf.write(CTRL_MEAS_REG, next_val);
                },
                .Temperature => {
                    const cur_val = try self.dev_intf.read(CTRL_MEAS_REG, 1);
                    const next_val = (cur_val & 0xE3) | (oversample_reg_val << 2);
                    try self.dev_intf.write(CTRL_MEAS_REG, next_val);
                },
            }
        }

        pub fn set_standby_time(self: bme280, time: StandbyTimeVal) !void {
            const time_reg_val: u8 = switch (time) {
                .HalfMs => 0x00,
                .SixtyTwoFiveMs => 0x01,
                .OneTwoFiveMs => 0x02,
                .TwoFiveZeroMs => 0x03,
                .FiveHundredMs => 0x04,
                .OneSecond => 0x05,
                .TenMs => 0x06,
                .TwentyMs => 0x07,
            };

            const cur_val = try self.dev_intf.read(CONFIG_REG, 1);
            const next_val = (cur_val & 0x8F) | (time_reg_val << 5);
            try self.dev_intf.write(CONFIG_REG, next_val);
        }

        pub fn set_filter_coef(self: bme280, filter_coef: FilterCoef) !void {
            const filter_coef_reg_val: u8 = switch (filter_coef) {
                .Off => 0x00,
                .Two => 0x01,
                .Four => 0x02,
                .Eight => 0x03,
                .Sixteen => 0x04,
            };

            const cur_val = try self.dev_intf.read(CONFIG_REG, 1);
            const next_val = (cur_val & 0xE3) | (filter_coef_reg_val << 4);
            try self.dev_intf.write(CONFIG_REG, next_val);
        }

        pub fn probe(self: bme280) !bool {
            const id_val = try self.dev_intf.read(ID_REG, 1);
            return id_val == ID_VAL;
        }

        pub fn get_calibration(self: bme280) Calibration {
            return self.calibration;
        }

        fn query_calibration(dev_intf: dev_intf_type) !Calibration {
            const calib1_reading = try dev_intf.read(CALIB1_REG_START, CALIB1_REG_END - CALIB1_REG_START);
            const calib2_reading = try dev_intf.read(CALIB2_REG_START, CALIB2_REG_END - CALIB2_REG_START);

            const ret: Calibration = .{
                .temperature = .{
                    .dig_T1 = byte_concat16(u16, calib1_reading[1], calib1_reading[0]),
                    .dig_T2 = byte_concat16(i16, calib1_reading[3], calib1_reading[2]),
                    .dig_T3 = byte_concat16(i16, calib1_reading[5], calib1_reading[4]),
                },

                .pressure = .{
                    .dig_P1 = byte_concat16(u16, calib1_reading[7], calib1_reading[6]),
                    .dig_P2 = byte_concat16(i16, calib1_reading[9], calib1_reading[8]),
                    .dig_P3 = byte_concat16(i16, calib1_reading[11], calib1_reading[10]),
                    .dig_P4 = byte_concat16(i16, calib1_reading[13], calib1_reading[12]),
                    .dig_P5 = byte_concat16(i16, calib1_reading[15], calib1_reading[14]),
                    .dig_P6 = byte_concat16(i16, calib1_reading[17], calib1_reading[16]),
                    .dig_P7 = byte_concat16(i16, calib1_reading[19], calib1_reading[18]),
                    .dig_P8 = byte_concat16(i16, calib1_reading[21], calib1_reading[20]),
                    .dig_P9 = byte_concat16(i16, calib1_reading[23], calib1_reading[22]),
                },

                .humidity = .{
                    .dig_H1 = calib1_reading[25],
                    .dig_H2 = byte_concat16(i16, calib2_reading[1], calib2_reading[0]),
                    .dig_H3 = calib2_reading[2],
                    .dig_H4 = (@as(i16, calib2_reading[3]) << 4) | (calib2_reading[4] & 0x0F),
                    .dig_H5 = (@as(i16, calib2_reading[5]) << 4) | (calib1_reading[4] >> 4),
                    .dig_H6 = calib2_reading[6],
                },
            };

            return ret;
        }

        pub fn get_all_measurements(self: bme280) !Data {
            const calib_data = self.calibration;
            const reg_data = try self.dev_intf.read(PRESS_MSB_REG, HUM_LSB_REG - PRESS_MSB_REG + 1);
            const press_data: i32 = (@as(i32, reg_data[0]) << 12) | (@as(i32, reg_data[1]) << 4) | (reg_data[5] >> 4);
            const temp_data: i32 = (@as(i32, reg_data[3]) << 12) | (@as(i32, reg_data[4]) << 4) | (reg_data[5] >> 4);
            const hum_data: u16 = byte_concat16(u16, reg_data[6], reg_data[7]);

            const temperature = calc_temperature(temp_data, calib_data.temperature);
            return .{
                .pressure = calc_pressure(press_data, temperature, calib_data.pressure),
                .temperature = temperature,
                .humidity = calc_humidity(hum_data, temperature, calib_data.humidity),
            };
        }

        pub fn is_measuring(self: bme280) !bool {
            const status_reg = try self.dev_intf.read(STATUS_REG, 1);
            return status_reg & 0x80 != 0;
        }

        pub fn reset(self: bme280) !void {
            try self.dev_intf.write(STATUS_REG, RESET_VAL);
        }

        fn calc_pressure(press_data: i32, temperature: f64, press_calib: Calibration.Pressure) f64 {
            const fpress_data: f64 = @floatFromInt(press_data);
            const t_fine = temperature * 5120.0;
            const dig_P1: f64 = @floatFromInt(press_calib.dig_P1);
            const dig_P2: f64 = @floatFromInt(press_calib.dig_P2);
            const dig_P3: f64 = @floatFromInt(press_calib.dig_P3);
            const dig_P4: f64 = @floatFromInt(press_calib.dig_P4);
            const dig_P5: f64 = @floatFromInt(press_calib.dig_P5);
            const dig_P6: f64 = @floatFromInt(press_calib.dig_P6);
            const dig_P7: f64 = @floatFromInt(press_calib.dig_P7);
            const dig_P8: f64 = @floatFromInt(press_calib.dig_P8);
            const dig_P9: f64 = @floatFromInt(press_calib.dig_P9);

            const var1 = t_fine / 2.0 - 64000.0;
            const var2 = var1 * var1 * dig_P6 / 32768.0;
            const var3 = var2 + var1 * dig_P5 * 2;
            const var4 = var3 / 4.0 + dig_P4 * 65536.0;
            const var5 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0;
            const var6 = (1.0 + var5 / 32768.0) * dig_P1;
            const p1 = 1048576.0 - fpress_data;
            const p2 = (p1 - (var4 / 4096.0)) * 6250.0 / var6;
            const var7 = dig_P9 * p2 * p2 / 2147483648.0;
            const var8 = p2 * dig_P8 / 32768.0;

            return p2 + (var7 + var8 + dig_P7) / 16.0;
        }

        fn calc_temperature(temp_data: i32, temp_calib: Calibration.Temperature) f64 {
            const ftemp_data: f64 = @floatFromInt(temp_data);
            const dig_T1: f64 = @floatFromInt(temp_calib.dig_T1);
            const dig_T2: f64 = @floatFromInt(temp_calib.dig_T2);
            const dig_T3: f64 = @floatFromInt(temp_calib.dig_T3);
            const var1: f64 = (ftemp_data / 16384.0 - dig_T1 / 1024.0) * dig_T2;
            const var2 = (ftemp_data / 131072.0 - dig_T1 / 8192.0) * (ftemp_data / 131072.0 - dig_T1 / 8192.0) * dig_T3;
            return (var1 + var2) / 5120.0;
        }

        fn calc_humidity(hum_data: u16, temperature: f64, hum_calib: Calibration.Humidity) f64 {
            const fhum_data: f64 = @floatFromInt(hum_data);
            const t_fine = temperature * 5120.0;
            const dig_H1: f64 = @floatFromInt(hum_calib.dig_H1);
            const dig_H2: f64 = @floatFromInt(hum_calib.dig_H2);
            const dig_H3: f64 = @floatFromInt(hum_calib.dig_H3);
            const dig_H4: f64 = @floatFromInt(hum_calib.dig_H4);
            const dig_H5: f64 = @floatFromInt(hum_calib.dig_H5);
            const dig_H6: f64 = @floatFromInt(hum_calib.dig_H6);

            const h1 = t_fine - 76800;
            const h2 = (fhum_data - (dig_H4 * 64.0 + dig_H5 / 16384.0 * h1));
            const h3 = h2 * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * h1 * (1.0 + dig_H3 / 67108864.0 * h1)));
            return h3 * (1.0 - dig_H1 * h3 / 542488.0);
        }
    };
}
