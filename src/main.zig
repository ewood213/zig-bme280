const std = @import("std");
const bme280 = @import("bme280.zig");
const print = std.debug.print;

pub fn main() !void {
    const sensor = try bme280.bme280.init();
    defer sensor.deinit();

    const probe_success = try sensor.probe();
    print("Probe success: {}\n", .{probe_success});
    const calib = try sensor.get_calibration();
    try sensor.set_oversampling(bme280.SensorType.Humidity, bme280.OverSamplingVal.One);
    try sensor.set_oversampling(bme280.SensorType.Pressure, bme280.OverSamplingVal.One);
    try sensor.set_oversampling(bme280.SensorType.Temperature, bme280.OverSamplingVal.One);
    try sensor.set_standby_time(bme280.StandbyTimeVal.OneSecond);
    try sensor.set_filter_coef(bme280.FilterCoef.Off);
    try sensor.put_mode(bme280.SensorMode.Normal);
    // std.time.sleep(2000000000);
    const data = try sensor.get_all_measurements(calib);
    print("Data: {}\n", .{data});
}
