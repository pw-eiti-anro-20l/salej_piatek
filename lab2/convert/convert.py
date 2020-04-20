#! /usr/bin/python

import json
from tf.transformations import *

xaxis,zaxis = (1, 0, 0),(0, 0, 1)
def convertToFile():
    with open('../convert/dh.json', 'r') as file:
        param = json.loads(file.read())

    with open('../convert/urdf.yaml', 'w') as file:
        for key in param.keys():
            a, d, alpha, th = param[key]   
            a=float(a)
            d=float(d)
            alpha=float(alpha)
            th=float(th)

            tz = translation_matrix((0, 0, d))  
            rz = rotation_matrix(th, zaxis)     
            tx = translation_matrix((a, 0, 0))  
            rx = rotation_matrix(alpha, xaxis)  

            matrix = concatenate_matrices(tz, rz, tx, rx) 

            rpy = euler_from_matrix(matrix)
            xyz = translation_from_matrix(matrix)

            file.write(key + ":\n")
            file.write("  j_xyz: {} {} {}\n".format(*xyz))
            file.write("  j_rpy: {} {} {}\n".format(*rpy))
            file.write("  l_xyz: 0 0 {}\n".format(xyz[2] / 2))
            file.write("  l_rpy: 0 0 0\n")
            file.write("  l_len: {}\n".format(d))
if __name__ == '__main__':
    param = {}
    results = ''
    convertToFile()
