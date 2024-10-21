# Simulador de UAVs V3

# Diagrama de arbol del modelo de Simulink

1. **UAV**
    1. **AC STATE CREATOR**
    2. **Environment Model**
    3. **Platform**
        1. **Aircraft Mechanics**
            1. **AC Bus Creator**
            2. **Forces & Moments Calculation**
                1. **Aerodynamic Forces and Moments**
                2. **Aerodynamic Coefficients**
                    1. **Aerodynamic Actuators**
                    2. **Longitudinal Aerodynamics (Cx, Cz, Cm coefficients)**
                        1. **Stability Axis Aerodynamics**
                        2. **Stability to Body Axis Matrix Computation**
                    3. **Actuator Increments**
                        1. **Aileron**
                        2. **Elevator**
                        3. **Rudder**
                        4. **Flap**
                    4. **Body Rate Damping**
                        1. **p Coefficients**
                        2. **q Coefficients**
                        3. **r Coefficients**
                    5. **alpha-beta Coefficients**
                3. **Propulsion**
                4. **Gravity**
            3. **6DOF (Quaternion)**
        2. **Air Data System**
    4. **Signal Conditioning**
    

# Nomenclatura utilizada (frames, variables de estado, fuerzas y momentos)

| Símbolo | Descripción |
| --- | --- |
| $X^eY^eZ^e \equiv \textbf i^e,\textbf j^e, \textbf k^e \equiv e\equiv NED$ | Ejes inerciales, navegación o North-East-Down |
| $X^bY^bZ^b \equiv \textbf i^b,\textbf j^b, \textbf k^b \equiv b$ | Ejes cuerpo |
| $X^wY^wZ^w \equiv \textbf i^w,\textbf j^w, \textbf k^w \equiv w$ | Ejes viento |
| $X^sY^sZ^s \equiv \textbf i^s,\textbf j^s, \textbf k^s \equiv s$ | Ejes estabilidad |
| $p_n$ | Posición inercial del UAV en dirección norte ($\textbf i^e$) |
| $p_e$ | Posición inercial del UAV en dirección este ($\textbf j^e$) |
| $p_d$ | Posición inercial del UAV en dirección hacia la superficie ($\textbf k^e$) (opuesto a la altitud) |
| $u$ | Velocidad inercial en la dirección del morro del UAV en $\textbf i^b$  |
| $v$ | Velocidad inercial en la dirección lateral $\textbf j^b$ |
| $w$ | Velocidad inercial en dirección hacia la superficie en vuelo horizontal $\textbf k^b$ |
| $\phi$ | Ángulo de Alabeo (Roll) |
| $\theta$ | Ángulo de Cabeceo (Pitch) |
| $\psi$ | Ángulo de Guiñada (Yaw) |
| $p$ | Roll Rate alrededor del eje $\textbf  i^b$ |
| $q$ | Roll Rate alrededor del eje $\textbf  j^b$ |
| $r$ | Roll Rate alrededor del eje $\textbf  k^b$ |
| $X$ | Fuerzas Externas Totales en el eje $\textbf i^b$ |
| $Y$ | Fuerzas Externas Totales  en el eje $\textbf j^b$ |
| $Z$ | Fuerzas Externas Totales  en el eje $\textbf k^b$ |
| $l$ | Momentos Externos Totales  en el eje $\textbf i^b$ |
| $m$ | Momentos Externos Totales en el eje $\textbf j^b$ |
| $n$ | Momentos Externos Totales  en el eje $\textbf k^b$ |

# Buses de Datos

## Control BUS (CTRL BUS)

Maneja las señales de control del avión en unidades del sistema internacional y con saturación.

| Nombre | Símbolo | Unidades | Descripción |
| --- | --- | --- | --- |
| deltaa  |  $\delta_a$ | rad | deflexión de los alerones |
| deltae  |  $\delta_e$  | rad | deflexión del elevador |
| deltar  |  $\delta_r$ | rad |  deflexión del rudder |
| deltaf  |  $\delta_f$  | rad |  deflexión de los flaps |
| deltat  |  $\delta_t$  | - | palanca de gases (de 0 a 1) |

## Air Data BUS (AD BUS)

Maneja las señales asociadas a el flujo del aire incidente, utilizadas para el cálculo de las fuerzas aerodinámicas.

| Nombre | Símbolo | Unidades | Descripción |
| --- | --- | --- | --- |
| alpha  |  $\alpha$  | rad | Ángulo de Ataque (AOA) |
| beta  | $\beta$  | rad | Sideslip Angle |
| Va  | $|\textbf V_a|=V_a$ | m/s | Módulo de la Velocidad Aerodinámica |
| pqr_tot | $\pmb\omega^b_{b/e} =[p,q,r]^T_b$ | rad/s | Velocidad Angular Total |
| q_bar | $\bar q =\dfrac{1}{2}\rho V^2$ | Pa | Presión Dinámica |
| Va - Aerodynamic Velocity | $\textbf V_a^b = [u_a,v_a,w_a]^T_b$ | m/s | Velocidad Aerodinámica |

## Propulsion Data BUS (PD BUS)

Maneja las señales asociadas al sistema propulsivo del avión. En esta versión del simulador la única señal no nula es la del empuje, debido a que no se posee un modelo para las RPM del motor del UAV y se ha supuesto un torque nulo alrededor del eje $X^b$.

| Nombre | Símbolo | Unidades | Descripción |
| --- | --- | --- | --- |
| RPM |  $\text{RPM}$ | - | Revoluciones por minuto del motor |
| Thrust | $T$  | N | Empuje |
| Torque | $Q$ | Nm | Torque del motor |

## Wind BUS (WIND BUS)

Maneja las señales relativas al viento generadas en el bloque **Wind Models.**

| Nombre | Símbolo | Unidades | Descripción |
| --- | --- | --- | --- |
| uvw_wind |  $\textbf V_w^b =[u_w, v_w, w_w]^T_b$ | m/s | Velocidad del viento respecto a tierra (Ejes Cuerpo) |
| pqr_wind | $\pmb{\omega}_{w/e}^b = [p_w, q_w, r_w]^T_b$  | rad/s | Velocidad Angular del viento respecto a tierra (Ejes Cuerpo) |

## Environment BUS (ENV BUS)

Maneja las señales asociadas al modelo ambiental sobre el que se desenvuelve el UAV (es decir, información relativa al modelo gravitatorio y al modelo atmosférico). 

| Nombre | Símbolo | Unidades | Descripción |
| --- | --- | --- | --- |
| g |  $\textbf{g}^e=[0, 0 , g]^T_e$ | m/(s*s) | Aceleración gravitatoria |
| rho | $\rho$  | N | Densidad del aire |
| Wind BUS | - | - | - |

## Aircraft BUS (AC BUS)

Maneja **todos los datos del simulador**. Integra a todos los demás buses con los datos provenientes del bloque **6DOF (Quaternion)** relativos a la dinámica de la aeronave (estados del avión). 

| Nombre | Símbolo | Unidades | Descripción |
| --- | --- | --- | --- |
| xyh  | $[p_n, p_e, h]=[p_n, p_e, -p_d]$ | m | Posición en el modelo de tierra plana |
| xdot ydot zdot  |  $\mathbf V_g^e=[\dot p_n,  \dot p_e, \dot p_d]^T_e$ | m/s | Velocidad Inercial (Ejes NED) |
| uvw | $\textbf V_g^b = [u,v,w]^T_b$ | m/s | Velocidad Inercial (Ejes Cuerpo) |
| hdot  |  $\dot h$  | m/s | Climb Rate |
| phi theta psi | $[\phi, \theta, \psi]^T$ | rad | Ángulos de Euler |
| DCMbe | $\textbf{DCM}_e^b$ | - | Matriz de Coseno Directores ($e\rightarrow b$) |
| pqr | $\pmb\omega^b_{b/e} =[p,q,r]^T_b$ | rad/s | Velocidad Angular Total |
| Control BUS | - | - | - |
| Air Data BUS | - | - | - |
| Propulsion Data BUS | - | - | - |
| Environment BUS | - | - | - |


## Modelo Propulsivo - Propulsion

COLOCAR AQUI IMAGEN DE LA TEORIA DE FROUDE

- El modelo propulsivo se basa en la ***Teoría Ideal de Froude,** pero aun más simplificada**.*** Se considera la hélice un disco infinitamente delgado de superficie $S_{prop}$ a través del cual existe una diferencia de presiones. Esta diferencia de presiones define la tracción de la hélice, y debe de suministrarse una cierta potencia a la misma para mantener esta diferencia de presiones. Se realizan las siguientes hipótesis:
    - Se asume que toda la energía suministrada a la hélice se transfiere al aire (hélice sin masa, indeformable).
    - Se asume que la velocidad no presenta discontinuidad a través del disco, se asume que la hélice no ofrece resistencia al paso del aire.
    - Hipótesis del principio de Bernoulli (fundamental en la aerodinámica potencial):
        - Flujo unidimensional a través de la vena fluida.
        - Flujo estacionario ($\frac{\partial}{\partial t} =0$).
        - Flujo incompresible ($\ \rho(x,y,z,t)=\text{const}\$ ).
        - Flujo no viscoso ( $\mu\approx 0$ ). No se tiene en cuenta la resistencia aerodinámica de las palas de la hélice real.
    
    Este modelo predice que el empuje es igual a la variación de la cantidad de movimiento del flujo de aire, y posee la siguiente expresión:
    
    $T =\dot m_{\text{air}} (V_{\text{exit}}-V_a) = \left(P_{\text{downstream}}-P_{\text{upstream}}\right)\cdot S_{\text{prop}}$
    
    donde:
    
    - $V_{\text{exit}}$: velocidad del flujo unidimensional aguas abajo cuando el fluido se expande y recupera la presión de remanso $P_0$, de forma que posee una presión $P_{\text{downstream}}=P_0+\frac{1}{2}\rho V_{\text{exit}}^2$.
    - $V_a$: velocidad del flujo sin perturbar aguas arriba. Si la atmósfera se encuentra estática (en ausencia de viento), la presión del fluido es $P_{\text{upstream}} = P_0+\frac{1}{2}\rho V_a^2$.
    - $\rho$ : densidad del aire.
- Se asume que la velocidad del fluido inmediatamente después de la hélice es directamente proporcional a la señal de control PWM, es decir: $V_{\text{exit}} =k_{\text{motor}}\delta_t$. La constante de proporcionalidad se puede determinar mediante experimentación.
- Se añade un factor de correción $C_{\text{prop}}$  que permite capturar de forma aproximada los efectos de las irreversibilidades despreciadas en las hipótesis anteriores.
- Finalmente el model de empuje utilizado queda como:
    
    $\boxed{T = \dfrac{1}{2}\rho S_{\text{prop}}C_{\text{prop}}\left[\left(k_{\text{motor}}\delta_t\right)^2-V_a^2\right]}$
    
- Debido a que no es posible estimar las RPM del motor, se toma como hipótesis que el torque del mismo es despreciable frente a los demás torques actuando sobre el eje $X^b$.
    
    $\boxed{Q\approx0}$
    
 
# Condicionado de las señales de control - Control BUS Creator

Este bloque se encarga de crear el bus de datos `CTRL BUS` , preparando las señales de entrada (PWM en rangos entre 0 y 1, y entre -1 y 1) para ser implementadas en la ecuaciones dinámicas.

- Las señales PWM de los alerones, elevador y rudder (`Ailerons Command (-1,1)`, `Elevator Command (-1,1)`, `Rudder Command (-1,1)`) se multiplican por $\delta_{\text{max}}>0, \textbf[\delta_{\text{max}}\textbf ]\equiv\text{rad}$, de forma que se generan las señales $\delta_a , \delta_e, \delta_r \in (-\delta_{\text{max}},\delta_{\text{max}})$. Este parámetro determina la deflexión máxima de las superficies de control en radianes. Se especifica en el archivo de configuración del modelo `config_model.m` .
- La señal PWM de los flaps puede ir de 0 a 1. Los flaps tienen un bloque de saturación y un rate limiter para emular la variación progresiva del ángulo de los flaps $\delta_f$. Además las posiciones de los flaps están limitadas a 0%, 20%, 60% o  100% de la deflexión máxima posible, es decir $\delta_f\in[0,\ 0.2\times \delta_{\text{max}},\  0.6\times  \delta_{\text{max}},\  \delta_{\text{max}}]$.
- La señal de la palanca de potencia del motor (throttle)  solo es pasada por un saturador para prevenir que esta escape del rango permitido, $\delta_t \in (0,1)$.
- Finalmente se crea el bus de control `CTRL BUS` .


# Modelos atmosférico y gravitatorio - Environment Model

El modelo ambiental utilizado consta de dos submodelos: el modelo gravitatorio (**WGS84 Gravity Model**) y el modelo atmosférico. A su vez, el modelo atmosférico está subdivido en un modelo de atmósfera estática para la determinación de la densidad del aire (**COESA Atmosphere Density Model**) y un modelo de viento (**Wind Models**).


## Modelo Gravitatorio -  **WGS84 Gravity Model**

- El modelo gravitatorio se basa en la distribución uniforme de la masa (equipotencial) del modelo elipsoidal de la tierra WGS84 (World Geodetic System 1984) utilizado para modelar la forma de la Tierra y definir coordenadas de ubicaciones geográficas en su superficie (desarrollado y mantenido por el DoD de los EEUU y la Agencia Nacional de Inteligencia Geoespacial). El tipo de modelo gravitatorio seleccionado es: **********Type of gravity model: WGS84 Taylor Series********** ya que no se necesita exagerada precisión en el modelo para trabajar con drones de corto alcance y baja altitud.
- Se utiliza el bloque **Flat Earth to LLA** para pasar de las coordenadas en ejes tierra/NED $X^eY^eZ^e$ del modelo de tierra plana $[p_n, p_e, p_d]^T$, a coordenadas geodésicas $[\mu, l, h]^T$ donde las variables $\mu, l, h$ se corresponden con la latitud, longitud y altitud de la plataforma.
- La entrada $[\mu, l, h]$ y la salida $[0, 0, g(\mu,l,h)]$ del bloque **WGS84 Gravity Model** son vectores fila (row). Por esta razón se colocan bloques de transposición a la entrada y salida del bloque (ya que todos los vectores en el espacio manejados en el simulador son de dimensión $3\times1$).

## Modelo de Viento - Wind Models

- La velocidad de la aeronave vista desde tierra $\textbf V_g$ (ground velocity) es la suma de la velocidad del avión respecto a la masa de aire circundante $V_a$, también llamada velocidad aerodinámica (air velocity) más la velocidad de dicha más de aire respecto a tierra $\textbf V_w$, es decir, la velocidad del viento (wind velocity).
    
    $\textbf V_g=\textbf V_a+\textbf V_w$
    
- La “airspeed” es el módulo o magnitud de la velocidad aerodinámica: $|\textbf V_a|=V_a=\sqrt{u_a^2+v_a^2+w_a^2}\equiv\text{airspeed}$.
- La velocidad del viento puede descomponerse en una componente determinista y medible $\textbf V_{w_s}$ (steady wind $\partial/\partial t=0$) y una aleatoria $\textbf V_{w_g}$ (random gusts, turbulences):
   