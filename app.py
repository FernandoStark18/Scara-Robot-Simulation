# Espericueta Fonseca Fernando Simón
# Mecatrónica 6to 6
# Cinemática de robots
# Análisis cinemático por la convención Denavit-Hartenberg del robot SCARA con 4 GDL


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from matplotlib import cm
from matplotlib.widgets import Slider

fig, ax = plt.subplots()
plt.subplots_adjust(left = 0, bottom = 0.3, right =1, top = 1)
ax = plt.axes(projection = "3d")

#Función para rotar la función en el eje x
def matriz_rotacion_y(grados):
	rad = grados/180*np.pi
	#Matriz de rotación
	rotacion=np.array([[np.cos(rad),0,-np.sin(rad),0],
					   [0,1,0,0],
					   [np.sin(rad),0,np.cos(rad),0],
					   [0,0,0,1]])
	return rotacion
#Función para rotar la función en el eje y
def matriz_rotacion_x(grados):
	rad = grados/180*np.pi
	#Matriz de rotacuón
	rotacion=np.array([[1,0,0,0],
					   [0,np.cos(rad),-np.sin(rad),0],
					   [0,np.sin(rad),np.cos(rad),0],
					   [0,0,0,1]])
	return rotacion
def matriz_rotacion_z(grados):
	rad = grados/180*np.pi
	#Matriz de rotacuón
	rotacion=np.array([[np.cos(rad),-np.sin(rad),0,0],
					   [np.sin(rad),np.cos(rad),0,0],
					   [0,0,1,0],
					   [0,0,0,1]])
	return rotacion

def matriz_traslacion_x(x):
	traslacion = np.array([[1,0,0,x],
						   [0,1,0,0],
						   [0,0,1,0],	   
						   [0,0,0,1]])
	return traslacion

def matriz_traslacion_y(y):
	traslacion = np.array([[1,0,0,0],
						   [0,1,0,y],
						   [0,0,1,0],	   
						   [0,0,0,1]])
	return traslacion

def matriz_traslacion_z(z):
	traslacion = np.array([[1,0,0,0],
						   [0,1,0,0],
						   [0,0,1,z],	   
						   [0,0,0,1]])
	return traslacion

def configuracion_grafica():
	plt.title("Robot 4 GDL RRRP", x = 0, y = 27)
	ax.set_xlim(-10,10)
	ax.set_ylim(-10,10)
	ax.set_zlim(-10,10)
	ax.set_xlabel("x")
	ax.set_ylabel("y")
	ax.set_zlabel("z")
	ax.view_init(elev=25,azim=30)

def sistema_coordenadas(a,b,c,a_1,b_1,c_1):
	x = [a,a_1]
	y = [b,b_1]
	z = [c,c_1]
	ax.plot3D(x,[b,b],[c,c],color='red')
	ax.plot3D([a,a],y,[c,c],color='blue')
	ax.plot3D([b,a],[b,b],z,color='green')

def sistema_coordenadas_movil(matriz_rotacion):
	r_11 = matriz_rotacion[0,0]
	r_12 = matriz_rotacion[1,0]
	r_13 = matriz_rotacion[2,0]
	r_21 = matriz_rotacion[0,1]
	r_22 = matriz_rotacion[1,1]
	r_23 = matriz_rotacion[2,1]
	r_31 = matriz_rotacion[0,2]
	r_32 = matriz_rotacion[1,2]
	r_33 = matriz_rotacion[2,2]

	dx = matriz_rotacion[0,3]
	dy = matriz_rotacion[1,3]
	dz = matriz_rotacion[2,3]


	ax.plot3D([dx,dx+r_11],[dy,dy+r_22],[dz,dz+r_13], color='m')
	ax.plot3D([dx,dx+r_21],[dy,dy+r_22],[dz,dz+r_23], color='c')
	ax.plot3D([dx,dx+r_31],[dy,dy+r_32],[dz,dz+r_33], color='y')
	plt.draw()

def DH(theta_i, di, ai, alpha_i):
	MT = matriz_rotacion_z(theta_i)@matriz_traslacion_z(di)@matriz_traslacion_x(ai)@matriz_rotacion_x(alpha_i)
	#MT = matriz_rotacion_x(theta_i)@matriz_traslacion_x(di)@matriz_traslacion_y(ai)@matriz_rotacion_y(alpha_i)
	return MT

def Robot_RRRP(theta_1, d1, a1, alpha_1, theta_2, d2, a2, alpha_2, theta_3, d3, a3, alpha_3, theta_4, d4, a4, alpha_4):
	A0 = np.eye(4)
	_0A1 = DH(theta_1, d1, a1, alpha_1)
	_1A2 = DH(theta_2, d2, a2, alpha_2)
	_2A3 = DH(theta_3, d3, a3, alpha_3)
	_3A4 = DH(theta_4, d4, a4, alpha_4)
	_0A1 = A0@_0A1
	_0A2 = _0A1@_1A2
	_0A3 = _0A2@_2A3
	_0A4 = _0A3@_3A4

	sistema_coordenadas_movil(A0)
	sistema_coordenadas_movil(_0A1)
	sistema_coordenadas_movil(_0A2)
	sistema_coordenadas_movil(_0A3)
	sistema_coordenadas_movil(_0A4)

	ax.plot3D([A0[0,3],_0A1[0,3]],[A0[1,3],_0A1[1,3]],[A0[2,3],_0A1[2,3]], color = 'red')
	ax.plot3D([_0A1[0,3],_0A2[0,3]],[_0A1[1,3],_0A2[1,3]],[_0A1[2,3],_0A2[2,3]], color = 'green')
	ax.plot3D([_0A2[0,3],_0A3[0,3]],[_0A2[1,3],_0A3[1,3]],[_0A2[2,3],_0A3[2,3]], color = 'red')
	ax.plot3D([_0A3[0,3],_0A4[0,3]],[_0A3[1,3],_0A4[1,3]],[_0A3[2,3],_0A4[2,3]], color = 'green')

def actualizacion_juntas(val):
		ax.cla()
		configuracion_grafica()
		theta_1 = sld_ang_1.val
		theta_2 = sld_ang_2.val
		theta_3 = sld_ang_3.val
		d4 = sld_d4.val

		Robot_RRRP(theta_1,0,10,0  ,theta_2,0,7,0  ,theta_3,-2,0,0  ,0,d4,0,0)
		plt.draw()
		plt.pause(1e-3)

ax1 = plt.axes([0.2,0.19,0.65,0.03])
ax2 = plt.axes([0.2,0.15,0.65,0.03])
ax3 = plt.axes([0.2,0.11,0.65,0.03])
ax4 = plt.axes([0.2,0.07,0.65,0.03])

sld_ang_1 = Slider(ax1, "Theta_1",-180,180,valinit = 0)
sld_ang_2 = Slider(ax2, "Theta_2",-180,180,valinit = 0)
sld_ang_3 = Slider(ax3, "Theta_3",-180,180,valinit = 0)
sld_d4 = Slider(ax4, "D4",-3,0,valinit = 0)

configuracion_grafica()
sld_ang_1.on_changed(actualizacion_juntas)
sld_ang_2.on_changed(actualizacion_juntas)
sld_ang_3.on_changed(actualizacion_juntas)
sld_d4.on_changed(actualizacion_juntas)
#Robot_RR(45,0,10,0  ,90,0,7,0  ,0,-2,0,0  ,0,-3,0,0)
plt.show()

print("Programado por Fernando Espericueta")