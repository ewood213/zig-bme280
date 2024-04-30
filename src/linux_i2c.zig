const std = @import("std");
const libc = @cImport({
    @cInclude("fcntl.h");
    @cInclude("sys/ioctl.h");
    @cInclude("linux/i2c-dev.h");
    @cInclude("unistd.h");
});

// Errors
pub const I2cError = error{ ReadError, WriteError, InitError };

// Types
pub fn create_i2c_interface_type(i2c_abspath: []const u8, address: u8) type {
    return struct {
        const i2c_interface = @This();
        const i2c_path = i2c_abspath;
        const i2c_address = address;
        file: std.fs.File,

        pub fn init() !i2c_interface {
            const file = try std.fs.openFileAbsolute(i2c_path, std.fs.File.OpenFlags{ .mode = .read_write });
            if (libc.ioctl(file.handle, libc.I2C_SLAVE, i2c_address) < 0) {
                return I2cError.InitError;
            }
            return i2c_interface{ .file = file };
        }

        pub fn deinit(self: i2c_interface) void {
            self.file.close();
        }

        fn get_read_type(comptime len: u32) type {
            if (len == 1) {
                return u8;
            } else {
                return [len]u8;
            }
        }

        pub fn read(self: i2c_interface, reg: u8, comptime len: u32) I2cError!get_read_type(len) {
            if (libc.write(self.file.handle, &reg, @sizeOf(@TypeOf(reg))) < 0) {
                return I2cError.WriteError;
            }
            var ret: get_read_type(len) = undefined;
            if (libc.read(self.file.handle, &ret, len) < 0) {
                return I2cError.ReadError;
            }
            return ret;
        }

        pub fn write(self: i2c_interface, reg: u8, val: u8) !void {
            const vals: [2]u8 = .{ reg, val };
            if (libc.write(self.file.handle, &vals, @sizeOf(@TypeOf(vals))) < 0) {
                return I2cError.ReadError;
            }
        }
    };
}
