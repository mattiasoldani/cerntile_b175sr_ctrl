import datetime
import serial
import sys
from time import sleep

port = "/dev/ttyACM0"
inpath = "./data/input/"
infilename = "path_default.txt"
outpath = "./data/output/"
dt = 0.2
t0 = datetime.datetime(1970, 1, 1)

print("---")

# reading input file with scan coordinates --> prepare string for serial
print("will send in the scan program:")
fullstr = "<"
with open(inpath+infilename, "r") as f:
    for line in f:
        fullstr += str(line) + ","
fullstr = fullstr.replace("\n","") + ">"
print(fullstr.encode())

print("---\n\n---")

# connecting to system
print("trying to connect port %s..." % port)
ser = serial.Serial(port, baudrate=9600, stopbits=1, timeout=1)
sleep(2)
ser.reset_input_buffer()
print("done!")

print("---\n\n---")

# sending string to system via serial
print("sending in scan program now...")
ser.write(fullstr.encode())
ser.flush()
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

        # open output file at start 
        if iloop == 0:
            filenamefull_loc = outpath+"%d.txt"%epoch
            print("all data will be written locally in %s" % filenamefull_loc)
            print("---\n\nvvv starting now... vvv")
            outfile = open(filenamefull_loc, "w")

        # current readings
        reads = [ser.readline()]
        while ser.in_waiting:
            reads.append(ser.readline())

        for read0 in reads:
            read = str(read0.decode()).replace("\n","").replace("\r","")
            if (len(read)==0): continue;

            # score output
            outstr = "[%d] %s" % (epoch, read)
            print(outstr)
            outfile.write(outstr+"\n")
            outfile.flush()
            iloop += 1
        sleep(dt)

    except KeyboardInterrupt:
        print("\n^^^^^^^^^^^^^^^^^^^^^^^")
        print("CTRL+C --> closing file, exiting...")
        outfile.close()
        sys.exit()
