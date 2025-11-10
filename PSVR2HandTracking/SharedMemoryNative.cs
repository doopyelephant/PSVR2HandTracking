using System;
using System.ComponentModel;
using System.IO.MemoryMappedFiles;
using System.Runtime.InteropServices;

internal static class SharedMemoryNative
{
    private const uint FILE_MAP_READ = 0x0004;

    [DllImport("kernel32.dll", SetLastError = true)]
    private static extern IntPtr MapViewOfFile(
        IntPtr hFileMappingObject,
        uint dwDesiredAccess,
        uint dwFileOffsetHigh,
        uint dwFileOffsetLow,
        UIntPtr dwNumberOfBytesToMap);

    [DllImport("kernel32.dll", SetLastError = true)]
    private static extern bool UnmapViewOfFile(IntPtr lpBaseAddress);

    public static IntPtr MapRead(MemoryMappedFile mmf, long offset = 0, long size = 0)
    {
        IntPtr hMap = mmf.SafeMemoryMappedFileHandle.DangerousGetHandle();
        uint high = (uint)((ulong)offset >> 32);
        uint low = (uint)((ulong)offset & 0xFFFFFFFF);
        UIntPtr length = size <= 0 ? UIntPtr.Zero : (UIntPtr)size; // 0 -> to end
        IntPtr p = MapViewOfFile(hMap, FILE_MAP_READ, high, low, length);
        if (p == IntPtr.Zero)
            throw new Win32Exception(Marshal.GetLastWin32Error(), "MapViewOfFile failed");
        return p;
    }

    public static void Unmap(IntPtr p)
    {
        if (p != IntPtr.Zero)
            UnmapViewOfFile(p);
    }
}