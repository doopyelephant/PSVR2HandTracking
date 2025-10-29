using System;
using System.IO;
using System.IO.MemoryMappedFiles;
using System.Threading;
using BCnEncoder.Encoder;
using BCnEncoder.Decoder;
using BCnEncoder.ImageSharp;
using BCnEncoder.Shared;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;


public static class Program
{
    private static long[] _offsets = new long[]
    {
        0x10BA00,
        0x30BB00,
        0x50BC00,
        0x70BD00,
        0x90BE00,
        0xB0BF00,
        0xD0C000,
        0xF0C100
    };

    private static MemoryMappedFile _shareFile;
    private static EventWaitHandle _imageEvent;
    private static MemoryMappedViewStream _imageViewStream;
    private static int _index = 0;

    public static void Main()
    {
        _shareFile = MemoryMappedFile.OpenExisting("SHARE_VRT2_WIN");
        _imageEvent = new EventWaitHandle(false, EventResetMode.AutoReset, "SHARE_VRT2_WIN_IMAGE_EVT");
        Update();
        Exit();
    }

    private static void Exit()
    {
        _shareFile.Dispose();
    }

    private static void Update()
    {
        _imageEvent.WaitOne();
            _imageViewStream = _shareFile.CreateViewStream(_offsets[_index], 0xFE100);
            byte[] buffer = new byte[0xFE100];
            _imageViewStream.Read(buffer, 0, 0xFE100);
            var dataleft = new byte[0x7F000];
            Array.Copy(buffer, 0x100, dataleft, 0, 0x7F000);
            var dataright = new byte[0x7F000];
            Array.Copy(buffer, 0x100 + 0x7F000, dataright, 0, 0x7F000);
            _imageViewStream.Dispose();
// Create a decoder
            var decoder = new BcDecoder();

            // Decode the image
            // This example assumes you know the width and height of the original image
            Image<Rgba32> decodedImage = decoder.DecodeRawToImageRgba32(dataleft,1024,1000,CompressionFormat.Bc4);
            Console.WriteLine("after decode");
            // Now you can work with the uncompressed image data
            // e.g., save it as a PNG, process pixel values
            decodedImage.SaveAsPng("decoded_bc4_image.png");
    }
}