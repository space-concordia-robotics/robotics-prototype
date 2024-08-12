"""
Script for generating Aruco marker images.

Author: Nathan Sprague
Version: 10/26/2020
"""

import argparse
import cv2
import numpy as np


class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter,
                      argparse.RawDescriptionHelpFormatter):
    """ Trick to allow both defaults and nice formatting in the help. """
    pass


def addWhiteBorder(image, borderSize):
    """Add a white border of the given number of pixels around the image.

    Parameters:
    image  (np.array, type np.uint8, 2D): black and white image
    borderSize: Number of pixels to add around each edge of the image

    Returns:
    np.array: image with border
    """

    newSize = (image.shape[0] + borderSize * 2, image.shape[1] + borderSize * 2)

    imageWithBorder = np.full(newSize, 255, dtype=image.dtype)

    imageWithBorder[borderSize:-borderSize, borderSize:-borderSize] = image

    return imageWithBorder


def main():
    parser = argparse.ArgumentParser(formatter_class=CustomFormatter,
                                     description="Generate a .png image of a specified maker.")
    parser.add_argument('--id', default=1, type=int,
                        help='Marker id to generate')
    parser.add_argument('--size', default=200, type=int,
                        help='Side length in pixels')
    parser.add_argument('--border', default=20, type=int,
                        help='White border size in pixels')
    
    dict_options = [s for s in dir(cv2.aruco) if s.startswith("DICT")]
    option_str = ", ".join(dict_options)
    dict_help = "Dictionary to use. Valid options include: {}".format(option_str)
    parser.add_argument('--dictionary', default="DICT_5X5_250", type=str,
                        choices=dict_options,
                        help=dict_help, metavar='')
    args = parser.parse_args()

    dictionary_id = cv2.aruco.__getattribute__(args.dictionary)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    image = np.zeros((args.size, args.size), dtype=np.uint8)

    if cv2.__version__ < "4.7.0":
        image = cv2.aruco.drawMarker(dictionary, args.id, args.size, image, 1)
    else:
        image = cv2.aruco.generateImageMarker(dictionary, args.id, args.size, image, 1)

    image = addWhiteBorder(image, args.border)

    cv2.imwrite("marker_{:04d}.png".format(args.id), image)



if __name__ == "__main__":
    main()
