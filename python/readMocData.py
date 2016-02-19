import yaml
import cv2
import numpy as na

from glob import glob
from base64 import b64decode
import zlib
import struct

class GaussianMapChannel:
    binarySize = 40
    unpacker = struct.Struct('ddddd')

    @staticmethod
    def fromBinary(data):
        assert len(data) == GaussianMapChannel.binarySize, (len(data), GaussianMapChannel.binarySize)
        unpackedTuple = GaussianMapChannel.unpacker.unpack(data)
        #print "tuple", unpackedTuple
        return GaussianMapChannel(*unpackedTuple)

    def __init__(self, counts, squaredcounts, mu, sigmasquared, samples):
        self.counts = counts
        self.squaredcounts = squaredcounts
        self.mu = mu
        self.sigmasquared = sigmasquared
        self.samples = samples
    def __repr__(self):
        return "GaussianMapChannel(%.10f, %.10f, %.10f, %.10f, %.10f)" % (self.counts, self.squaredcounts,
                                         self.mu, self.sigmasquared,
                                         self.samples)

class GaussianMapCell:
    binarySize = GaussianMapChannel.binarySize * 4

    @staticmethod
    def fromBinary(data):
        assert len(data) == GaussianMapCell.binarySize
        return GaussianMapCell(GaussianMapChannel.fromBinary(data[0:GaussianMapChannel.binarySize]),
                               GaussianMapChannel.fromBinary(data[GaussianMapChannel.binarySize:GaussianMapChannel.binarySize * 2]),
                               GaussianMapChannel.fromBinary(data[GaussianMapChannel.binarySize * 2:GaussianMapChannel.binarySize * 3]),
                               GaussianMapChannel.fromBinary(data[GaussianMapChannel.binarySize * 3:GaussianMapChannel.binarySize * 4]))

    def __init__(self, red, green, blue, z):
        self.red = red
        self.green = green 
        self.blue = blue
        self.z = z
    def __repr__(self):
        return "GaussianMapCell(%s, %s, %s)" % (repr(self.red), repr(self.green), repr(self.blue))

class GaussianMap:
    @staticmethod
    def fromYaml(yamlGmap):


        cellData = readBinaryFromYaml(yamlGmap["cells"])
        print "cellData", len(cellData)

        expectedSize = yamlGmap["width"] * yamlGmap["height"] * GaussianMapCell.binarySize
        assert len(cellData) == expectedSize, (len(cellData), expectedSize)
        
        cells = []
        for i in range(yamlGmap["width"] * yamlGmap["height"]):
            cell = GaussianMapCell.fromBinary(cellData[i * GaussianMapCell.binarySize:(i + 1) * GaussianMapCell.binarySize])
            cells.append(cell)
        return GaussianMap(yamlGmap["width"], yamlGmap["height"],
                           yamlGmap["x_center_cell"], yamlGmap["y_center_cell"],
                           yamlGmap["cell_width"], cells)

    def __init__(self, width, height, x_center_cell, y_center_cell, cell_width, cells):
        self.width = width
        self.height = height
        self.x_center_cell = x_center_cell
        self.y_center_cell = y_center_cell
        self.cell_width = cell_width
        self.cells = cells
        
    def toImage(self):
        data = []
        for x in range(0, self.width):
            row = []
            for y in range(0, self.height):
                cell = self.cells[x  + self.width * y]
                row.append((cell.blue.mu, cell.green.mu, cell.red.mu))
                #print "row", row
            data.append(row)
        imgYCC = na.array(data, dtype=na.float32) / 255.0
        imgRGB = cv2.cvtColor(imgYCC, cv2.COLOR_YCR_CB2BGR)
        return imgRGB

def readBinaryFromYaml(yamlList):
    data = "".join(yamlList)
    decoded = b64decode(data)
    decompressed = zlib.decompress(decoded)
    return decompressed

doubleunpacker = struct.Struct('d')
uintunpacker = struct.Struct('I')
def readMatFromYaml(fs):
    rows = fs["rows"]
    cols = fs["cols"]
    imgtype = fs["type"]
    print "type", imgtype
    m = cv2.cv.CreateMat(rows, cols, imgtype)
    
    if imgtype == 6:
        numpytype = na.float32
        unpacker = doubleunpacker
        #numpytype = na.uint32
        #unpacker = uintunpacker
    else:
        raise ValueError("Unknown image type: " + repr(imgtype))
    array = na.zeros(m.rows * m.cols * m.channels, dtype=numpytype)
    binary = readBinaryFromYaml(fs["data"])
    size = unpacker.size
    for i in range(len(array)):
        data = unpacker.unpack(binary[i*size:(i+1)*size])
        assert len(data) == 1
        array[i] = data[0]
    array = na.transpose(array.reshape((m.rows, m.cols, m.channels)), axes=(1,0,2))
    return array

def main():

    for fname in glob("left/*/*/ein/sceneModel/model.yml"):
        print fname

        f = open(fname) 

        lines = []
        # ignore the %YAML:1.0, because the python parser doesn't handle 1.0.
        f.readline() 

        for line in f:
            # for some reason the python parser doesn't like this line either.
            if "background_pose" in line:
                continue
            lines.append(line)
        data = "\n".join(lines)
        
        ymlobject = yaml.load(data)
        #print ymlobject
        scene = ymlobject["Scene"]
        observed_map = GaussianMap.fromYaml(scene["observed_map"])
        image = observed_map.toImage()
        cv2.imwrite("observed.png", image)
        cv2.imshow("observed map", image)
        
        
        
        
        print "observed map: ", observed_map.width, "x", observed_map.height
        dimage = readMatFromYaml(scene["discrepancy_magnitude"])
        cv2.imshow("discrepancy magnitude", dimage)

        cv2.waitKey(0)
        cv2.destroyAllWindows()

        #cells = scene["cells"]
        #print "cells", len(cells)


if __name__ == "__main__":
    main()
