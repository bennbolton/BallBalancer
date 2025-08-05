import numpy as np
from scipy import linalg
from matplotlib import pyplot as plt

 #https://github.com/nliaudat/magnetometer_calibration/blob/main/calibrate.py
 #corrected code S. James Remington
 
class Magnetometer(object):
    
    '''
	To obtain Gravitation Field (raw format):
    1) get the Total Field for your location from here:
       http://www.ngdc.noaa.gov/geomag-web (tab Magnetic Field)
       es. Total Field = 47,241.3 nT | my val :47'789.7
    2) Convert this values to Gauss (1nT = 10E-5G)
       es. Total Field = 47,241.3 nT = 0.47241G
    3) Convert Total Field to Raw value Total Field, which is the
       Raw Gravitation Field we are searching for
       Read your magnetometer datasheet and find your gain value,
       Which should be the same of the collected raw points
       es. on HMC5883L, given +_ 1.3 Ga as Sensor Field Range settings
           Gain (LSB/Gauss) = 1090 
           Raw Total Field = Gain * Total Field
           0.47241 * 1090 = ~515  |
           
        -----------------------------------------------
         gain (LSB/Gauss) values for HMC5883L
            0.88 Ga => 1370 
            1.3 Ga => 1090 
            1.9 Ga => 820
            2.5 Ga => 660 
            4.0 Ga => 440
            4.7 Ga => 390 
            5.6 Ga => 330
            8.1 Ga => 230 
        -----------------------------------------------
	
	
     references :
        -  https://teslabs.com/articles/magnetometer-calibration/      
        -  https://www.best-microcontroller-projects.com/hmc5883l.html

    '''
    MField = 330 #Gravitation Field : TO CHANGE according the previous formula

    def __init__(self, F=MField): 


        # initialize values
        self.F   = F
        self.b   = np.zeros([3, 1])
        self.A_1 = np.eye(3)
        
    def run(self):
        data = np.loadtxt("mag_out.txt", delimiter=',')
        print("shape of data array:", data.shape)
        print("First 5 rows raw:\n", data[:5])


        print("Raw axis min/max:")
        print("X:", np.min(data[:,0]), np.max(data[:,0]))
        print("Y:", np.min(data[:,1]), np.max(data[:,1]))
        print("Z:", np.min(data[:,2]), np.max(data[:,2]))

        # Ellipsoid fit
        s = np.array(data).T
        M, n, d = self.__ellipsoid_fit(s)

        # Calibration parameters
        M_1 = linalg.inv(M)
        self.b = -np.dot(M_1, n)
        # self.A_1 = np.real(self.F / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) * linalg.sqrtm(M))
        U, S, Vt = np.linalg.svd(M)
        scale = np.cbrt(self.F / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d))  # cube root to keep volume
        self.A_1 = np.dot(U, np.dot(np.diag(np.sqrt(S)), Vt)) * scale

        print("\nDiagonal of A_1:", np.diag(self.A_1))
        print("Norm of A_1:", np.linalg.norm(self.A_1))

        print("\nData normalized to ", self.F)
        print("Soft iron transformation matrix:\n", self.A_1)
        print("Hard iron bias:\n", self.b)

        result = []
        for row in data:
            v = row.reshape(3, 1) - self.b  # subtract bias
            v = np.dot(self.A_1.T, v)       # apply transpose of A_1
            result.append(v.flatten())

        result = np.array(result)

        # 3D plot: raw vs calibrated
        fig = plt.figure(figsize=(12, 6))
        ax1 = fig.add_subplot(121, projection='3d')
        ax1.set_title("Raw Magnetometer Data")
        ax1.scatter(data[:, 0], data[:, 1], data[:, 2], color='red', s=4)
        ax1.set_xlabel("X"); ax1.set_ylabel("Y"); ax1.set_zlabel("Z")
        ax1.grid(True)

        ax2 = fig.add_subplot(122, projection='3d')
        ax2.set_title("Calibrated Data (Should Be a Sphere)")
        ax2.scatter(result[:, 0], result[:, 1], result[:, 2], color='green', s=4)
        ax2.set_xlabel("X"); ax2.set_ylabel("Y"); ax2.set_zlabel("Z")
        ax2.grid(True)

        plt.tight_layout()
        plt.show()

        print("First 5 rows calibrated:\n", result[:5])

        # Save corrected data to file "out.txt"
        np.savetxt('out.txt', result, fmt='%f', delimiter=' ,')

        print("*************************")
        print("code to paste (edits required) :")
        print("*************************")
        self.b = np.round(self.b, 2)
        print("float B[3] = {", self.b[0], ",", self.b[1], ",", self.b[2], "};\n")

        self.A_1 = np.round(self.A_1, 5)
        print("float A_inv[3][3] = {")
        print("{", self.A_1[0, 0], ",", self.A_1[0, 1], ",", self.A_1[0, 2], "},")
        print("{", self.A_1[1, 0], ",", self.A_1[1, 1], ",", self.A_1[1, 2], "},")
        print("{", self.A_1[2, 0], ",", self.A_1[2, 1], ",", self.A_1[2, 2], "}};")
        print("\n")


        # Calculate the range (max - min) for each axis
        def axis_range(data):
            return np.max(data, axis=0) - np.min(data, axis=0)

        raw_range = axis_range(data)
        calibrated_range = axis_range(result)

        print("Raw data axis ranges (X, Y, Z):", raw_range)
        print("Calibrated data axis ranges (X, Y, Z):", calibrated_range)



    def __ellipsoid_fit(self, s):
        ''' Estimate ellipsoid parameters from a set of points.

            Parameters
            ----------
            s : array_like
              The samples (M,N) where M=3 (x,y,z) and N=number of samples.

            Returns
            -------
            M, n, d : array_like, array_like, float
              The ellipsoid parameters M, n, d.

            References
            ----------
            .. [1] Qingde Li; Griffiths, J.G., "Least squares ellipsoid specific
               fitting," in Geometric Modeling and Processing, 2004.
               Proceedings, vol., no., pp.335-340, 2004
        '''

        # D (samples)
        D = np.array([s[0]**2., s[1]**2., s[2]**2.,
                      2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
                      2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])

        # S, S_11, S_12, S_21, S_22 (eq. 11)
        S = np.dot(D, D.T)
        S_11 = S[:6,:6]
        S_12 = S[:6,6:]
        S_21 = S[6:,:6]
        S_22 = S[6:,6:]

        # C (Eq. 8, k=4)
        C = np.array([[-1,  1,  1,  0,  0,  0],
                      [ 1, -1,  1,  0,  0,  0],
                      [ 1,  1, -1,  0,  0,  0],
                      [ 0,  0,  0, -4,  0,  0],
                      [ 0,  0,  0,  0, -4,  0],
                      [ 0,  0,  0,  0,  0, -4]])

        # v_1 (eq. 15, solution)
        E = np.dot(linalg.inv(C),
                   S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

        E_w, E_v = np.linalg.eig(E)

        v_1 = E_v[:, np.argmax(E_w)]
        if v_1[0] < 0: v_1 = -v_1

        # v_2 (eq. 13, solution)
        v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)

        # quadratic-form parameters, parameters h and f swapped as per correction by Roger R on Teslabs page
        M = np.array([[v_1[0], v_1[5], v_1[4]],
                      [v_1[5], v_1[1], v_1[3]],
                      [v_1[4], v_1[3], v_1[2]]])
        n = np.array([[v_2[0]],
                      [v_2[1]],
                      [v_2[2]]])
        d = v_2[3]

        return M, n, d
        
        
        
if __name__=='__main__':
        Magnetometer().run()