import numpy as np
import math

def Rvec2Rmat(vec):
    mod = np.linalg.norm(vec)
    unit_vec = vec / mod

    theta = mod
    i = np.identity(3)

    temp = np.array([[0, -float(unit_vec[2]), float(unit_vec[1])],
                    [float(unit_vec[2]), 0, -float(unit_vec[0])],
                    [-float(unit_vec[1]), float(unit_vec[0]), 0]])

    Rmat = math.cos(theta) * i + (1 - math.cos(theta)) * unit_vec * np.transpose(unit_vec) + math.sin(theta) * temp

    return Rmat
    

def main():
    vec = np.array([[-0.91402138],
       [ 0.90976494],
       [ 1.36800709]])
    Rmat = Rvec2Rmat(vec)
    print(Rmat)

if __name__ == "__main__":
    main()