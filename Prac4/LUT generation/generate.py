import numpy as np
from scipy import signal

import matplotlib.pyplot as plt

def main():

    lutLen = 500

    t = np.linspace(0, 1, lutLen, endpoint=False)

    saw = signal.sawtooth(2 * np.pi * t, 1)
    tri = signal.sawtooth(2 * np.pi * t, 0.5)
    sine = np.sin(2 * np.pi * t )

    sine = convertToLut(sine)
    saw = convertToLut(saw)
    tri = convertToLut(tri)

    writeToFile(sine, "LUT generation/sine.lut")
    writeToFile(saw, "LUT generation/saw.lut")
    writeToFile(tri, "LUT generation/tri.lut")


    plt.plot(saw)
    plt.plot(tri)
    plt.plot(sine)

    plt.show()




def convertToLut(sig):
    return (  np.rint(  (  (sig + 1) / np.max(sig + 1)  ) * 1023  )  ).astype(int)


def writeToFile(sig, filePath):
    
    sigFile = open(filePath, "w")

    for i in range(np.shape(sig)[0]):
        
        if i == np.shape(sig)[0] - 1:
            sigFile.write("{:4d}".format(sig[i]))
        else:
            sigFile.write("{:4d},".format(sig[i]))
            if (i+1)%16 == 0:
                sigFile.write("\n")







if __name__ == "__main__":
    main()