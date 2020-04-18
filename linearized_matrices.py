import numpy as np


def linear_matrices():
	r = 0.045
	m = 0.9
	M = 1
	b = 0.01
	c = 0.0047
	J = 5e-6
	I = 1e-3
	alpha = 0
	g = -9.8
	l = 0.07

	n1_32 = M*g*l*(J + 2*M*(r**2) + 2*m*(r**2) - M*(r**2)*(np.cos(alpha)**2) - m*(r**2)*(np.cos(alpha)**2) + M*l*r*np.cos(alpha))
	d1_32 = M**2*l**2*r**2 - M**2*l**2*r**2*(np.cos(alpha)**2) + m*M*l**2*r**2 + J*M*l**2 + I*M*r**2 + I*m*r**2 + I*J
	n2_32 = 4*M**2*l**2*r**3*g*np.cos(alpha)*np.sin(alpha)*(M+m)*(2*M*np.sin(alpha)*l**2 + 2*M*r*l*np.cos(alpha)*np.sin(alpha) +2*M*r*l*np.cos(alpha)*np.sin(alpha) + 2*I*np.sin(alpha));
	d2_32 = (2*I*J + M**2*l**2*r**2 + 2*J*M*l**2 + 2*I*M*r**2 + 2*I*m*r**2 - (M**2*l**2*r**2)*(2*np.cos(alpha)**2 - 1) + 2*M*(l**2)*m*(r**2))**2

	a32 = (n1_32/d1_32) + (n2_32/d2_32)

	n33 = b*(I + J + M*l**2 + M*r**2 + m*r**2 + 2*M*l*r*np.cos(alpha))
	d33 = M**2*l**2*r**3 - M**2*l**2*r**3*np.cos(alpha)**2 + m*M*l**2*r**3 + J*M*r*l**2 + I*M*r**3 + I*m*r**3 + I*J*r
	a33 = -n33/d33

	n34 = I*b + J*b - c*m*r**3 - J*c*r + M*b*l**2 + M*b*r**2 - M*c*r**3 + b*m*r**2 - M*c*l*r**2*np.cos(alpha) + 2*M*b*l*r*np.cos(alpha)
	d34 = r*(M**2*l**2*r**2 - M**2*l**2*r**2*(np.cos(alpha)**2) + m*M*l**2*r**2 + J*M*l**2 + I*M*r**2 + I*m*r**2 + I*J)
	a34 = -n34/d34

	n1_42 = M*g*l*(J + M*r**2 + m*r**2 + M*r**2*np.sin(alpha)**2 + m*r**2*np.sin(alpha)**2)
	d1_42 = M**2*l**2*r**2*np.sin(alpha)**2 + m*M*l**2*r**2 + J*M*l**2 + I*M*r**2 + I*m*r**2 + I*J
	n2_42 = 2*M**3*g*l**3*r**4*np.sin(2*alpha)**2*(M + m)
	d2_42 = (2*I*J + M**2*l**2*r**2 + 2*J*M*l**2 + 2*I*M*r**2 + 2*I*m*r**2 + 2*M*l**2*m*r**2 - M**2*l**2*r**2*np.cos(2*alpha))**2
	a42 = (-n1_42/d1_42) - (n2_42/d2_42)

	n43 = J*b + M*b*r**2 + b*m*r**2 + M*b*l*r*np.cos(alpha)
	d43 = r*(M**2*l**2*r**2 - M**2*l**2*r**2*np.cos(alpha)**2 + m*M*l**2*r**2 + J*M*l**2 + I*M*r**2 + I*m*r**2 + I*J)
	a43 = n43/d43

	n44 = J*b - c*m*r**3 - J*c*r + M*b*r**2 - M*c*r**3 + b*m*r**2 + M*b*l*r*np.cos(alpha)
	d44 = r*(M**2*l**2*r**2 - M**2*l**2*r**2*(np.cos(alpha)**2) + m*M*l**2*r**2 + J*M*l**2 + I*M*r**2 + I*m*r**2 + I*J)
	a44 = n44/d44
	
	linA = np.array([[0, 0, 1, 0], [0, 0, 0, 1], [0, a32, a33, a34], [0, a42, a43, a44]])

	nb3 = I + J + M*l**2 + M*r**2 + m*r**2 + 2*M*l*r*np.cos(alpha)
	db3 = M**2*l**2*r**2 - M**2*l**2*r**2*np.cos(alpha)**2 + m*M*l**2*r**2 + J*M*l**2 + I*M*r**2 + I*m*r**2 + I*J
	b3 = nb3/db3

	nb4 = J + M*r**2 + m*r**2 + M*l*r*np.cos(alpha)
	db4 = M**2*l**2*r**2 - M**2*l**2*r**2*np.cos(alpha)**2 + m*M*l**2*r**2 + J*M*l**2 + I*M*r**2 + I*m*r**2 + I*J
	b4 = nb4/db4

	linB = np.array([0, 0, b3, b4])
	return linA, linB


if __name__ == "__main__":
	print("Linearized state transition and control matrices. Output:")
	A, B = linear_matrices()
	print("State matrix A: \n", A)
	print("Control matrix B: \n", B)
