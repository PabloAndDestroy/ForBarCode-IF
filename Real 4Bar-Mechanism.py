import numpy as np 
from math import radians, pi
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from collections import deque
#Librerías importadas
time=300 #Tiempo total de la animación
change= 0.002 #Cambio en el tiempo, o sea cada cuanto va a graficar 
w3_w2=np.zeros((time,2), dtype=float)
h=np.zeros((time,2), dtype=float) #Almacena Punto inicial
z=np.zeros((time,2), dtype=float) #Almacena posiciones l4
v=np.zeros((time,2), dtype=float) #Almacena posiciones l1
w=np.zeros((time,2), dtype=float) #Almacena posiciones l2
e=np.zeros((time,2), dtype=float) #Almacena posiciones l5 (punto final, arriba del triangulo) PUNTO P
animation = ()
angulos_maximos = ()
vpx_vpy=np.zeros((time,2), dtype=float) #Arreglo que almacena las velocidades del punto P
l1=float(input("Ingresar la barra numero 1: ")) #barra que gira
l2=float(input("Ingresar la barra numero 2: "))
l3=float(input("Ingresar la barra numero 3: ")) 
l4=float(input("Ingresar la barra numero 4: ")) #Marco
l5=float(input("Ingresar la barra numero 5: ")) #Triangulo fijo
th1=np.radians(int(input("Ingrese el angulo de entrada entre 0 y 90°: ")))
th5=np.radians(int(input("Ingresar el angulo de la barra 5: "))) #Triangulo fijo
w1= float(input("Ingrese la velocidad en rpm de la barra de entrada: "))
l6=np.sqrt((l5**2)+(l3**2)-(2*l5*l3*np.cos(radians(th5)))) #L6 con ley de coseno
th6=np.arccos(((l5**2)+(l3**2)-(l6**2))/(2*(l5*l3))) #Ley de cosenos
z[:,0]=l4
#t = np.arange(0, 2.5, 0.1)

barras=np.array([l1, l2, l3, l4]) #Lista de barras
barras.sort()
s=barras[0]
l=barras[-1]
q=barras[2]
p=barras[1]
L=l1+l2+l3+l4

def remove_atypical(x):  #Para remover desviaciones
    for i in range(len(x)):
        for j in range(len(x[i])):
           if x[i][j] > 50 or x[i][j]<-50:
               x[i][j] = np.NaN
    return x

def detectar_rotacion(bar):
   global barras, l1, l2, l3, l4, s, l, q, p
   s_l=barras[0]+barras[-1] #S más L
   p_q=barras[1]+barras[2] #P más Q
   #Identificar si es Grashoff o no
   if (s_l<p_q):
      print("Grashoff")
   elif (s_l>p_q):
      print("No Grashoff")
   if s_l <= p_q:
        if l2 == s and s_l != p_q:
          return "Coupler R" # Rotating Coupler case
        elif l1 == l and l3 == s and s_l != p_q:
          temp = l1
          l1 = l3
          l3 = temp
          return "Rocker3" # Manivela-balancín cuando l3 es balancín
        elif l4 == l1 and l2 == l3:
          return "DELTA" #Grashof Especial 
        elif (l4 == l or l1 == l) and s_l == p_q:
          return "PARAL"#Grashof Especial (Más problemático)
        else:
          return "CompleteRot"
   else:
        # Los casos No-Grashof no tienen manivelas
        if l1 == l or l4 == l:
            return "OutRocker"
        elif l2 == l or l3 == l:
            return "Tipo34"

def definir_angulos(tipo):
    if tipo == "CompleteRot":
        return np.linspace(0,np.radians(360) + (4*pi), time)
    if tipo == "Coupler R": # Caso Rotating Coupler
        th_max = np.arccos((l1**2+l4**2-(l2+l3)**2)/(2*l1*l4))
        th_min = np.arccos(((l1+l2)**2+l4**2-l3**2)/(2*(l1+l2)*l4))
        forward = np.linspace(th_min+change, th_max- change, time//2)
        backward = np.linspace(th_max-change, th_min + change, time//2)
        return np.concatenate([forward, backward])
    if tipo == "Rocker3":
        return np.linspace(0,np.radians(360) + (4*pi), time)
    if tipo == "OutRocker":
        alpha = np.arccos((-(l2 + l3) **2 + l4 ** 2 + l1 ** 2) / (2 * l4 * l1))
        th_min = (2 * pi) - alpha
        th_max = (2 * pi) + alpha 
        forward = np.linspace(th_min + change, th_max - change, time//2)
        backward = np.linspace(th_max - change, th_min + change, time//2)
        return np.concatenate([forward, backward])
    if tipo == "Tipo34":
        alpha = np.arccos((-(l2 - l3) ** 2 + l4 ** 2 + l1 ** 2) / (2 * l4 * l1))
        th_min = alpha
        th_max = (2 * pi) - alpha 
        forward = np.linspace(th_min + change, th_max - change, time//2)
        backward = np.linspace(th_max - change, th_min + change, time//2)
        return np.concatenate([forward, backward])
    if tipo == "DELTA":
        return np.linspace(th1, th1 + (4*pi), time)
    if tipo == "PARAL":
        th_max = np.arccos((l1 ** 2 + l4 ** 2 - (l2 + l3) ** 2) / (2 * l1 * l4))
        th_min = np.arccos(((l1 + l2) ** 2 + l4 ** 2 - l3 ** 2) / (2 * (l1 + l2) * l4))
        forward = np.linspace(th_min + change, th_max - change, time // 2)
        backward = np.linspace(th_max - change, th_min + change, time // 2)
        return np.concatenate([forward, backward])

   

def revolutes_and_vels(theta_main):
  global w1
  i=0
  K1 = l4 / l1 #Estas ecuaciones están en el libro https://lsbunefm.files.wordpress.com/2018/10/disec3b1o-de-maquinaria-robert-l-norton-4.pdf pag. 165
  K2 = l4 / l3 #Desde la pag 165, ellos usan a,b,c,d nosotros l1,l2,l3,l4
  K3 = (l4**2 + l1**2 - l2**2 + l3**2) / (2 * l1 * l3) 
  K4=l4/l2 #Estas dos nueva k4 y k5 sirven para los momentos donde un eslabón se cruza con otro
  K5=(-l4**2 - l1**2 - l2**2 + l3**2)/(2 * l1 * l2)
  for th1 in theta_main:
    A = np.cos(th1)*(1-K2)+K3-K1
    B = -2*np.sin(th1)
    C = -np.cos(th1)*(1+K2)+K3+K1
    D = np.cos(th1)*(1+K4)+K5-K1
    E = -2*np.sin(th1)
    F = np.cos(th1)*(K4-1)+K1+K5


    th2 = 2*np.arctan(((-E-np.sqrt(E**2-(4*D*F)))/(2*D))) #Estas ecuaciones deben servir más, solo q no sé en q momento hay q usar E- o E+ (mirad libro pagina 168)
    th3 = 2*np.arctan((-B-np.sqrt((B**2-(4*A*C)))/(2*A)))
    th2I = 2 * np.arctan(((-E + np.sqrt(E ** 2 - (4 * D * F))) / (
            2 * D)))  # Estas ecuaciones deben servir más, solo q no sé en q momento hay q usar E- o E+ (mirad libro pagina 168)
    th3I = 2 * np.arctan((-B + np.sqrt((B ** 2 - (4 * A * C))) / (2 * A)))

    # Crear linspace para los casos que requieran inversión geometrica (especiales y routating coupler
    tht2 = np.linspace(th2, th2I, time // 2)
    tht3 = np.linspace(th3, th3I, time // 2)
    # Se verá horrible pero no hubo de otra:
    if (s + l < p + q):
        if l2 == l or l3 == l:
            for z in tht2:
                l2_x = l2 * np.cos(z)
                l2_y = l2 * np.sin(z)
        else:
            l2_x = l2 * np.cos(th2)
            l2_y = l2 * np.sin(th2)
            l3_x = l3 * np.cos(th3)
            l3_y = l3 * np.sin(th3)
            l5_x = l5 * np.cos(th5 + th2)
            l5_y = l5 * np.sin(th5 + th2)

    if (s + l == p + q):
        for z in tht2:
            l2_x = l2 * np.cos(z)
            l2_y = l2 * np.sin(z)
            l5_x = l5 * np.cos(th5 + z)
            l5_y = l5 * np.sin(th5 + z)

        for z in tht3:
            l3_x = l3 * np.cos(z)
            l3_y = l3 * np.sin(z)
            l6_x = l6 * np.cos(th6 + z) #EL TRIANGULO QUEDA RARO PERO EL CASO FUNCIONA, FUNCIONAAAA
            l6_y = l6 * np.sin(th6 + z)


    else:
        l2_x = l2 * np.cos(th2)
        l2_y = l2 * np.sin(th2)
        l3_x = l3 * np.cos(th3)
        l3_y = l3 * np.sin(th3)
        l5_x = l5 * np.cos(th5 + th2)
        l5_y = l5 * np.sin(th5 + th2)
        l6_x = l6 * np.cos(th6)
        l6_y = l6 * np.sin(th6)

    #Componentes de las barras en cada instante de tiempo
    l1_x= l1*np.cos(th1)
    l1_y= l1*np.sin(th1)




    v[i, :]= np.array([l1_x,l1_y])
    w[i, :]= np.array([l1_x+l2_x, l1_y+l2_y])
    e[i, :]= np.array([(l1_x+l5_x), (l1_y+l5_y)])

    #Calculo de velocidades,  se hace con: x = A^-1 * C
    x_l3= l3*(np.cos((pi/2)+th3))
    x_l2= l2*(np.cos((pi/2)+th2+pi))
    y_l3= l3*(np.sin((pi/2)+th3))
    y_l2= l2*(np.sin((pi/2)+th2+pi))

    mat_A = np.array([[x_l3, x_l2] ,[y_l3, y_l2]])  #Lo guardamos en una matriz 
    mat_C = np.array([[w1*l1*(np.cos((pi/2)+th1)+0.0001)],[w1*l1*(np.sin((pi/2)+th1))]])

    inv_mat_A = np.linalg.inv(mat_A) #A^-1 * C

    w3_w2[i, :] = (inv_mat_A @ mat_C).reshape((1, 2))

    w2 = w3_w2[i, 1]
    w3 = w3_w2[i, 0]

    vpx = w1*l1*(np.cos(th1+(pi/2))) + w2*l5*(np.cos(th1+th5+(pi/2)))
    vpy = w1*l1*(np.sin(th1+(pi/2))) + w2*l5*(np.sin(th1+th5+(pi/2)))

    vpx_vpy[i, :] = np.array([vpx, vpy])

    i=i+1

def inicializar():
  line.set_data([], [])
  line2.set_data([], [])
  trace.set_data([], [])
  return line, line2, trace


def animacion(i):
  valsx = [h[i, 0], v[i, 0], w[i, 0], z[i, 0]]
  valsy = [h[i, 1], v[i, 1], w[i, 1], z[i, 1]]
  line.set_data(valsx, valsy)
  valsx = [v[i, 0], e[i, 0], w[i, 0]]
  valsy = [v[i, 1], e[i, 1], w[i, 1]]
  line2.set_data(valsx, valsy)
  #Traza
  if i == 0:
    Trace_x.clear()
    Trace_y.clear()

  Trace_x.appendleft(valsx[1])
  Trace_y.appendleft(valsy[1])

  trace.set_data(Trace_x, Trace_y)
  return line, line2, trace

tipo=detectar_rotacion(barras)
angulos=definir_angulos(tipo)
revolutes_and_vels(angulos)
remove_atypical(w3_w2) #Elimina datos atipicos
remove_atypical(vpx_vpy) #Elimina datos atipicos
all_coordinates= np.vstack([h,v,w,z,e])

maximo_xy = np.zeros((1, 2), dtype=float)
minimo_xy = np.zeros((1, 2), dtype=float)

maximo_xy[:, 0] = np.amax(all_coordinates[:, 0])
maximo_xy[:, 1] = np.amax(all_coordinates[:, 1])
minimo_xy[:, 0] = np.amin(all_coordinates[:, 0])
minimo_xy[:, 1] = np.amin(all_coordinates[:, 1])

fig = plt.figure()
ray = fig.add_subplot(111, aspect="equal", autoscale_on=False, xlim=(minimo_xy[:, 0] - 1, maximo_xy[:, 0] + 1), ylim=(minimo_xy[:, 1] - 1, maximo_xy[:, 1] + 1))
Trace_p = 300

#Add Grid Lines, Titles and Labels
ray.grid(alpha=0.5)
ray.set_title("Animación mecanismo")
ray.set_xticklabels([])
ray.set_yticklabels([])
(line, ) = ray.plot([], [], marker='o', lw=5, color="chocolate")
(line2, ) = ray.plot([], [], marker='o', lw=5, color="darkgreen")
(trace, ) = ray.plot([], [], '.-', lw=2, ms=2, color="slateblue")
Trace_x, Trace_y = deque(maxlen=Trace_p), deque(maxlen=Trace_p)
ani = anim.FuncAnimation(fig, animacion, init_func=inicializar, frames=len(e), interval=30, blit=True, repeat=True, save_count=1200)
#Plots
fig2, ((ray1, ray2), (ray3, ray4), (ray5, ray6)) = plt.subplots(3, 2)
fig2.suptitle('Análisis de posición y velocidad',fontsize=10)
ray1.plot(angulos, w3_w2[:, 1], color="#6F38C5")
ray1.set_title("Velocidad angular en P", fontsize=8)
ray1.set_xlabel('Theta1', fontsize=7)
ray1.set_ylabel('w2', fontsize=7)
ray1.grid(True)

ray2.plot(e[:, 0], e[:, 1], color="#C689C6")
ray2.set_title("Trayectoria en punto P", fontsize=8)
ray2.set_xlabel('Px', fontsize=7)
ray2.set_ylabel('Py', fontsize=7)
ray2.grid(True)

ray3.plot(angulos, e[:, 0], color="springgreen")
ray3.set_title("Coordenadas de Px",fontsize=8)
ray3.set_xlabel('Tht-1', fontsize=7)
ray3.set_ylabel('Px', fontsize=7)
ray3.grid(True)

ray4.plot(angulos, e[:, 1], color="orangered")
ray4.set_title("Coordenadas de Py",fontsize=8)
ray4.set_xlabel('Tht-1', fontsize=7)
ray4.set_ylabel('Py', fontsize=7)
ray4.grid(True)

ray5.plot(angulos, vpx_vpy[:, 0], color="red")
ray5.set_title("Velocidad lineal en Px", fontsize=8)
ray5.set_xlabel('Tht-1', fontsize=7)
ray5.set_ylabel('Vpx', fontsize=7)
ray5.grid(True)

ray6.plot(angulos, vpx_vpy[:, 1], color="salmon")
ray6.set_title("Velocidad lineal en Py",fontsize=8)
ray6.set_xlabel('Tht-1', fontsize=7)
ray6.set_ylabel('Vpy', fontsize=7)
ray6.grid(True)

plt.subplots_adjust(left=0.1,
                    bottom=0.1,
                    right=0.9,
                    top=0.9,
                    wspace=0.3,
                    hspace=0.9)

plt.show()