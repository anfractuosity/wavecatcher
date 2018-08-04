# Software

Edit the makefile, so that OPENCM3_DIR is set to the location of 
libopencm3.

You can clone that from https://github.com/libopencm3/libopencm3

Then run 'make'.

And use program.sh to program using a J-Link.

Use the following commands to generate/process/extract audio:

```
sudo ./wavecatcher
./process
sox -r 22000 -c 1 -b 8 -t raw -e unsigned-integer out.bin 1.wav
```

# Todo

Convert PDM to PCM in a more intelligent fashion!


