const std = @import("std");
const bme280 = @import("bme280.zig");
const linux_i2c = @import("linux_i2c.zig");
const print = std.debug.print;

const i2c_interface = linux_i2c.create_i2c_interface_type("/dev/i2c-1", bme280.ADDR_LOW);
const i2c_bme280 = bme280.create_bme280_type(i2c_interface);

const CONVERSION_DELAY_MS = 100 * 1000 * 1000;
const TIME_BETWEEN_SAMPLES_NS = 60 * 1000 * 1000 * 1000;

pub fn main() !void {
    const sensor = try init_probe();
    defer sensor.deinit();

    const stdout = std.io.getStdOut().writer();
    try stdout.print("Time,Temperature,Pressure,Humidity\n", .{});
    while (true) {
        const timestamp = std.time.timestamp();
        try sensor.put_mode(bme280.SensorMode.Forced);
        std.time.sleep(CONVERSION_DELAY_MS);
        const data = try sensor.get_all_measurements();
        try stdout.print("{},{},{},{}\n", .{ timestamp, data.temperature, data.pressure, data.humidity });
        std.time.sleep(TIME_BETWEEN_SAMPLES_NS - CONVERSION_DELAY_MS);
    }
}

fn init_probe() !i2c_bme280 {
    const sensor = try i2c_bme280.init();
    try sensor.set_oversampling(bme280.SensorType.Humidity, bme280.OverSamplingVal.One);
    try sensor.set_oversampling(bme280.SensorType.Pressure, bme280.OverSamplingVal.One);
    try sensor.set_oversampling(bme280.SensorType.Temperature, bme280.OverSamplingVal.One);
    try sensor.set_standby_time(bme280.StandbyTimeVal.OneSecond);
    try sensor.set_filter_coef(bme280.FilterCoef.Off);

    return sensor;
}
