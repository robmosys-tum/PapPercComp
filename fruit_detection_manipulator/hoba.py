import numpy as np

uv_1 = np.array([300,200,1])
uv_1 = uv_1.transpose()

newcam_mtx = np.array([
            [1.16392310e+03, 0.00000000e+00, 6.28041958e+02],
            [0.00000000e+00, 1.12263977e+03, 3.42072773e+02],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
 ])

inverse_newcam_mtx = np.linalg.inv(newcam_mtx)

tvec1 = np.array([
        -10.44832422,
        -13.4489327,
        5.22469801
    ])

R_mtx= np.array([
        [ 0.99863679, -0.05182086, -0.00625726],
        [ 0.05206021,  0.99751354,  0.04750242],
        [ 0.00378008, -0.04776342,  0.99885152]
    ])

inverse_R_mtx = np.linalg.inv(R_mtx)

scaling_factor = 49.59819

suv_1= np.multiply(scaling_factor, uv_1)
xyz_c= inverse_newcam_mtx.dot(suv_1)
xyz_c= np.subtract(xyz_c, tvec1.T)
XYZ= inverse_R_mtx.dot(xyz_c)
# XYZ*= 0.004

print(XYZ)





