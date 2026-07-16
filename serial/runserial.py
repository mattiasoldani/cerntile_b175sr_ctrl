import datetime
import os
import serial
import sys
from time import sleep

port = "/dev/ttyACM0"
dt = 1
t0 = datetime.datetime(1970, 1, 1)
filepath_loc = "./data/"
filesize = int(36000 / dt)
inpath = "./data/input/"
infile = "path_default.txt"

print("---")

# reading input file with scan coordinates --> prepare string for serial
print("will send in the scan program:")
fullstr = "<"
with open(inpath+infile, "r") as f:
    for line in f:
        fullstr += str(line)
fullstr = fullstr.replace("\n","") + ">"
print(fullstr.encode())

print("---\n\n---")

# connecting to system
print("trying to connect port %s..." % port)
ser = serial.Serial(port, baudrate=9600, stopbits=1)
print("done!")

print("---\n\n---")

# sending string to system via serial
print("sending in scan program now...")
ser.write(fullstr.encode())
print("done!")

print("---\n\n---")

print("starting readout loop...")
print("close it with CTRL+C")

iloop = 0
while True:
    try:

        # current timestamp
        t = datetime.datetime.now() - t0
        epoch = int(t.total_seconds())

        if iloop%filesize == 0:
            print("\nnext iteration (%d):" % iloop)

            # open output file(s)
            filenamefull_loc = filepath_loc + "%d.txt"%epoch
            print("all data will be written locally in %s" % filenamefull_loc)
            outfile_loc = open(filenamefull_loc, "w")

        # current reading
        ser.write(b':READ?\n') ; read0 = ser.readline()
        #read = float(read0)
        read = read0

        # score output and wait until next iteration
        outstr = "%d,%s" % (epoch, read)
        print(outstr)
        outfile_loc.write(outstr+"\n")
        outfile_loc.flush()

        if (iloop+1)%filesize == 0:
            outfile_loc.close()

        iloop += 1
        sleep(dt)

    except KeyboardInterrupt:
        print("\nCTRL+C --> Exiting...")
        sys.exit()
