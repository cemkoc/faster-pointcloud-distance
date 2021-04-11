import argparse
from laspy.file import File
import numpy as np
from pygeodesy.ellipsoids import Ellipsoids
from pygeodesy.ecef import EcefKarney
from pygeodesy.latlonBase import LatLonBase

LAS = "las"
PLY = "ply"
XYZ = "xyz"


def las_to_ecef(lasfile):
    X_invalid = np.logical_or((lasfile.header.min[0] > lasfile.x),
                              (lasfile.header.max[0] < lasfile.x))
    Y_invalid = np.logical_or((lasfile.header.min[1] > lasfile.y),
                              (lasfile.header.max[1] < lasfile.y))
    Z_invalid = np.logical_or((lasfile.header.min[2] > lasfile.z),
                              (lasfile.header.max[2] < lasfile.z))

    bad_indices_mask = np.logical_or(X_invalid, Y_invalid, Z_invalid)
    bad_indices_size = np.where(bad_indices_mask)[0].shape[0]

    print(f"Number of bad points to be removed: {bad_indices_size}")

    coords = np.vstack((lasfile.x, lasfile.y, lasfile.z)).T
    # only choose good indices
    coords = coords[bad_indices_mask == False]

    # convert GEODETIC coordinates (lon, lat, height) to ECEF
    ecef_converter = EcefKarney(Ellipsoids.WGS84)
    # coords is shaped (n, 3) for n number of points in lon, lat, height format
    ecef_coords = []
    for i in range(coords.shape[0]):
        latlon = LatLonBase(coords[i, 1], coords[i, 0], coords[i, 2])
        ecef_tuple = ecef_converter.forward(latlon)
        ecef_coords.append([ecef_tuple.x, ecef_tuple.y, ecef_tuple.z])

    return np.array(ecef_coords)


def write_ecef_to_ply(data, outfile):
    """
    Writes ECEF coordinates to PLY file with minimal headers.
    args:
    - data: np.ndarray representing ecef coordinate points
    - outfile: full file path to output PLY file.
    """
    assert data.shape[1] == 3, "ERROR: input data is not correctly shaped"

    data_size = len(data)
    header = f"ply\nformat ascii 1.0\nelement vertex {data_size}\nproperty float32 x\nproperty float32 y\nproperty float32 z\nend_header\n"

    print(f"Writing to file: {outfile}")
    with open(outfile, 'w') as fout:
        fout.write(header)
        for i in range(data_size):
            line = f"{data[i, 0]} {data[i, 1]} {data[i, 2]}\n"
            fout.write(line)

    print("Done.")


def main(argv):
    inputfile = argv.input
    inputformat = inputfile.split(".")[-1]

    print(f"Input file to process: {inputfile}")

    if inputformat == LAS:
        infile = File(inputfile, mode="r")
        ecef_coords = las_to_ecef(infile)

    else:
        raise NotImplementedError(
            f"Input file format: {format} not yet implemented.")

    write_ecef_to_ply(ecef_coords, outfile=inputfile.split(".")[0] + "." + PLY)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i",
                        "--input",
                        help="input file path that contains point cloud data")
    # parser.add_argument("-o", "--output", help="output file path")
    args = parser.parse_args()
    main(args)
