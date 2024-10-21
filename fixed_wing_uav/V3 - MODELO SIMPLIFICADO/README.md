# Simulador de UAVs (FWS)- V3

<aside>
⚠️ El Simulador de UAVs de Ala Fija (Fixed Wing Simulator, FWS) posee tres versiones con distintos grados de complejidad y detalle.

**V1.  VERSIÓN COMPLEJA.** 
       - *Modelo propulsivo eléctrico completo*. Modelo Motor + Modelo Hélice.
       - *Modelo aerodinámico completo.* Polar parabólica extendida: 
                                            $C_D(\alpha)= C_{D_p} + k_2C_L + k_1 C_L^2$
**V2. VERSIÓN INTERMEDIA.**
       *******- Modelo propulsivo simplificado.******* Motor por Lookup Table + Modelo Hélice.
       ************- Modelo aerodinámico completo.************
****V3. VERSIÓN SIMPLIFICADA.****
       ******- Modelo propulsivo muy simplificado. $T = \frac{1}{2}\rho S_{prop}C_{prop}\ [(k_{motor}\delta_t)^2-V_a^2]$*
       - ***********Modelo aerodinámico simplificado. $C_D(\alpha)=C_{D_p} + C_{D_\alpha}\alpha$

**********************************************************************************************La versión de la presente documentación es la V3. VERSION SIMPLIFICADA.***********************************************************************************************

</aside>

# Generalidades

## Realizar una simulación

Para simular un vuelo del UAV, es necesario abrir el modelo `simulador2017b.slx` . En este, se resuelve un *sistema de ecuaciones algebraica-diferenciales (DAEs, Differential Algebraic Equations)  no lineales*, para determinar las variables de estado $\mathbf x$ y las variables algebraicas $\mathbf x_a$ del vuelo. Es decir:

$\left\{ \begin{matrix}
\mathbf {\dot x }= f(\mathbf x,\mathbf x_a, t)\\
0 = g(\mathbf x,\mathbf x_a,t)
\end{matrix}\right.$

Estas variables se detallan más adelante. Ver la siguiente imagen (**X VEC**).

![Modelo de UAV de propulsión eléctrica con sus entradas y salidas. El modelo presenta una serie de instrumentos en la parte superior (*COCKPIT*) para comprobar el estado del vuelo mientras transcurre la simulación.](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled.png)

Modelo de UAV de propulsión eléctrica con sus entradas y salidas. El modelo presenta una serie de instrumentos en la parte superior (*COCKPIT*) para comprobar el estado del vuelo mientras transcurre la simulación.

Los valores de las entradas por defecto (*Contant Commands*) y los valores iniciales de las variables de estado, son los correspondientes al trimado de una vuelo en crucero a altitud constante $h=100m$. Los valores iniciales de los estados se establecen dentro de la Mask del bloque **UAV**. **Este modelo puede simularse directamente debido a que ya se encuentra trimado** en las condiciones anteriormente mencionadas.

![**UAV** Mask](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%201.png)

**UAV** Mask

Las entradas del sistemas se encuentran a la izquierda del bloque **UAV**, donde se especifica el nombre de cada entrada y el rango de valores que admite. ****Las salidas se encuentran a la derecha. Existen 4 salidas en total.

- **AC BUS.** Bus que maneja ************todos************ los datos del avión, incluyendo variables de estado $\mathbf x$ y variables algebraicas $\mathbf x_a$.
- ****************X LAT BUS.****************  Variables de estado de la dinámica lateral del avión $\mathbf x_{\text{lat}}$.
- ****************X LONG BUS.****************  Variables de estado de la dinámica lateral del avión $\mathbf x_{\text{long}}$.
- ****************X LAT BUS.****************  Variables de estado de la dinámica lateral del avión $\mathbf x=[\mathbf x_{\text{lat}},\mathbf x_{\text{long}}]$. Cabe destacar que el vector $x$  ************************************************************************************************************************************no se conforma utilizando lo concatenación mostrada anteriormente************************************************************************************************************************************ (solo se busca ilustrar que el mismo integra en si las variables tanto laterales como longitudinales)************************************************************************************************************************************.************************************************************************************************************************************

El bloque ********Scopes******** aloja una serie de monitores/scopes para visualizar los datos del vuelo en función del tiempo.

![Diagrama de bloques dentro del bloque **UAV**. Se tienen 4 bloques principales: **[Control BUS Creator**,](https://www.notion.so/Simulador-de-UAVs-FWS-V3-637d736e467345f5893f223daf4063ee?pvs=21) **AC State Creator**, **[Environment Model](https://www.notion.so/Simulador-de-UAVs-FWS-V3-637d736e467345f5893f223daf4063ee?pvs=21)** y **Platform**. El bloque **Platform** integra toda la dinámica del UAV asi como también modela el sistema de datos de aire (**Air Data System**) para el cálculo del AOA, side-slip angle y el módulo de la velocidad del avión respecto al aire (airspeed). El bloque ******************AC State Creator****************** es un bloque auxiliar que crea los buses de estado para un fácil manejo de estos datos. ](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%202.png)

Diagrama de bloques dentro del bloque **UAV**. Se tienen 4 bloques principales: **[Control BUS Creator**,](https://www.notion.so/Simulador-de-UAVs-FWS-V3-637d736e467345f5893f223daf4063ee?pvs=21) **AC State Creator**, **[Environment Model](https://www.notion.so/Simulador-de-UAVs-FWS-V3-637d736e467345f5893f223daf4063ee?pvs=21)** y **Platform**. El bloque **Platform** integra toda la dinámica del UAV asi como también modela el sistema de datos de aire (**Air Data System**) para el cálculo del AOA, side-slip angle y el módulo de la velocidad del avión respecto al aire (airspeed). El bloque ******************AC State Creator****************** es un bloque auxiliar que crea los buses de estado para un fácil manejo de estos datos. 

## Configurando un UAV específico

**Todos los parámetros del sistema/modelo,** se modifican desde el archivo `config_model.m` . Los únicos parámetros que se modifican directamente desde los menús contextuales (mask) de los bloques, son los del modelo de vientos.

Por defecto se tiene configurado el **Aerosonde UAV (**[AAI Aerosonde - Wikipedia](https://en.wikipedia.org/wiki/AAI_Aerosonde)**).** Los parámetros de este avión se encuentran disponibles en el libro “*[Small Unmanned Aircraft: Theory and Practice](https://github.com/randybeard/uavbook)*” (Randy Beard, Tim McLain):

![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%203.png)

![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%204.png)

El script de configuración se divide en diferentes secciones:

1. **Load Propulsion Data.** Se cargan las LookUp tables del empuje y torque de la hélice del vehículo (`data_PER310x6E_arrays.mat`) mediante el script `LOOKUP_TABLES.m` . **Estos datos no se utilizan en esta versión del simulador** pero serán utilizados en versiones posteriores. Además, cabe destacar que estos datos son extraidos de los datasheet del fabricante [APC Propellers | Quality Propellers that are Competition Proven](https://www.apcprop.com/) . Cada hélice del fabricante tiene asociada un archivo `.csv` con datos provenientes de ensayos experimentales. Estos datos deben ser procesados antes de poder ser utilizados (este proceso es el que genera el archivo `.mat` mencionado anteriormente). 
2. ********************************************************Geometry and mass properties********************************************************. Propiedades geométrica y másicas del UAV, como pueden ser: MTOW (`mass`), la superficie de referencia del modelo (`S`), la envergadura de las alas (`b`), la cuerda media del ala (`c_bar`) o el tensor de inercia (`J`).
3. **Simulation initial conditions and atmospheric conditions.**  Latitud y longitud iniciales del modelo para la determinación de la aceleración gravitatoria en dicho lugar, así como los parámetros que definen las ráfagas de viento que se deseen simular durante el vuelo. Para más detalles soibre el modelo de vientos y atmosférico ver **[Modelo de Viento - Wind Models](https://www.notion.so/Simulador-de-UAVs-FWS-V3-637d736e467345f5893f223daf4063ee?pvs=21)**
4. ************************Propulsion.************************ Parámetros propulsivos como la superficie frontal de la hélice (`Sprop`) o la constante propulsiva del modelo (`C_prop`).
5. **Aerodynamics.** Coeficientes adimensionales (derivadas de estabilidad) que definen todo el comportamiento aerodinámico del UAV.

## Trimado del modelo (cálculo de puntos de equilibrio)

El trimado del UAV consiste en hallar los estados de equilibrio $\mathbf x= \mathbf x_{\text{eq}}$ y variables algebraicas de equilibrio $\mathbf x_a= \mathbf x_{a,\text{eq}}$ que cumplan:

$\left\{ \begin{matrix}
\mathbf{\dot x_{\text{eq}}} = f(\mathbf x_{\text{eq}},\mathbf x_{a,\text{eq}}, t) 
= \mathbf C \\
g(\mathbf x_{\text{eq}},\mathbf x_{a,\text{eq}},t) =0
\end{matrix}\right.$

Es decir, es necesario resolver un sistema de ecuaciones algebraicas no lineales definido por el vector $\mathbf C$ **constante** y por los valores que se definan como conocidos dentro de $\mathbf x_{\text{eq}}$ y $\mathbf x_{a,\text{eq}}$ (siempre y cuando den pie a un sistema compatible y con solución).

<aside>
⚠️ Cabe destacar que el vector $\mathbf {\dot x_{\text{eq}} }=\mathbf C$ ******************************************no tiene que tener todas sus componentes nulas,****************************************** es decir, ******************************************************************************existen maniobras o puntos de operación tales que $\mathbf {\dot x_{\text{eq}} }=\mathbf C\ne0$.** Por ejemplo, en un ******************************************************************************ascenso con climb-rate constante****************************************************************************** se tiene que $\dot h = \text{const}\ne0\ \frac{\text{ft}}{\text{s}}$, siendo este un punto de operación (o equilibrio) perfectamente válido. Lo mismo puede suceder en un ******************************************************************************viraje uniforme a altura constante****************************************************************************** donde se cumple que $\dot \psi = \text{const} \ne 0 \ \frac{\text{rad}}{\text{s}},\ R(t)=\sqrt{p_n^2(t)+p_e^2(t)}=\text{const}, \ h=\text{const}$ (donde $p_n , p_e, h$ son componentes del estado de equilibrio $\mathbf{\dot x_{\text{eq}}}$). ************************************************************************************************************************************************************

</aside>

Los pasos recomendados para realizar el trimado del modelo son los siguientes:

1. Se recomienda realizar una copia del modelo, en la cual se sustituyan las entradas y salidas predeterminadas con *Inports* y *Outports* . Se suministra el modelo de trimado `simulador2017bTRIM.slx` para ahorrarnos este paso. Observe que se ha establecido un empuje del $50\%$ y flaps a $0\degree$, mientras que se han establecido como entradas variables los ángulos de alerones, elevador y rudder, de allí los bloques *Inport* (estos ángulos serán calculados por el programa de trimado). Ver la siguiente imagen:
    
    ![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%205.png)
    
2. Posteriormente, se abre la herramienta *Linear Analysis Tool*. Una forma de abrirla rápidamente es haciendo click derecho sobre el bloque **UAV** y luego, dentro del menú contextual seleccionando *Linear Analysis >Linearize Block…*
    
    ![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%206.png)
    
3. Se selecciona la opción *Trim Model.*
    
    ![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%207.png)
    
4. Finalmente, se configuran las variables que se desean en steady-state ($\dot x = 0 \Rightarrow x=\text{const})$, se establecen los valores iniciales para la búsqueda del punto de equilibrio (comunmente llamados *seeds.* En nuestro caso se establecen en la columna *****Value*****) y se imponen  los valores conocidos del punto de equilibrio (columna *know*). Además puede configurarse el rango de búsqueda de las soluciones mediante las columnas *Minumum* y *Maximum.* Para empezar el trimado se hace click en el botón *Start trimming.*
    
    ![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%208.png)
    

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

# Modelos de fuerzas y momentos - Forces & Moments Calculation

![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%209.png)

El bloque 6DOF (Quaternion) de Simulink recibe como entradas los vectores de fuerzas `Fxyz (N)` y momentos `Mxyz (N-m)` totales sobre el avión. Estos vectores son calculados en este bloque. 

Las ecuaciones utilizadas en el modelo en ejes cuerpo $X^bY^bZ^b$ son las siguientes:

$\boxed{
\begin{bmatrix}X\\Y\\Z\end{bmatrix} = mg\begin{bmatrix}-\sin\theta \\\cos\theta\sin\phi\\\cos\theta\cos\phi\end{bmatrix} + \dfrac{1}{2}\rho V_a^2S\begin{bmatrix}C_X(\alpha)+C_{X_q}(\alpha)(c/2V_a)\ q+C_{X_{\delta_e}}(\alpha)\delta_e\\C_{Y_0}+C_{Y_\beta}\beta + C_{Y_p}(b/2V_a)\ p+C_{Y_r}(b/2V_a)\ r +C_{Y_{\delta_r}}\delta_r+C_{Y_{\delta_a}}\delta_a\\C_Z(\alpha)+C_{Z_q}(\alpha)(c/2V_a)\ q+C_{Z_{\delta_e}}(\alpha)\delta_e\end{bmatrix}+\dfrac{1}{2}\rho S_{\text{prop}}C_{\text{prop}}\begin{bmatrix}(k_{\text{motor}}\delta_t)^2-V_a^2\\0\\0\end{bmatrix}
}$

$\boxed{
\begin{bmatrix}C_X(\alpha)&C_{X_q}&C_{X_{\delta_e}}\\C_Z(\alpha)&C_{Z_q}&C_{Z_{\delta_e}}\end{bmatrix}_{2\times3}= \begin{bmatrix}
\cos \alpha&-\sin \alpha\\
\sin \alpha&\cos \alpha
\end{bmatrix}_{2\times2} \begin{bmatrix}-C_L(\alpha)&-C_{L_q}&-C_{L_{\delta_e}}\\-C_D(\alpha)&-C_{D_q}&-C_{D_{\delta_e}}\end{bmatrix}_{2\times3}
}$ (Corresponde con 3 proyecciones de ejes $X^sZ^s$ a ejes $X^b Z^b$, una para cada vector columna).

$\boxed{
\begin{bmatrix}l\\m\\n\end{bmatrix} = \dfrac{1}{2}\rho V_a^2S\begin{bmatrix}

b\left[C_{l_0}+C_{l_\beta}\beta + C_{l_p}(b/2V_a)\ p+C_{l_r}(b/2V_a)\ r +C_{l_{\delta_r}}\delta_r+C_{l_{\delta_a}}\delta_a\right]

\\ c\left[C_m(\alpha)+C_{m_q}(c/2V_a)\ q+C_{m_{\delta_e}}\delta_e\right]

\\b\left[C_{n_0}+C_{n_\beta}\beta + C_{n_p}(b/2V_a)\ p+C_{n_r}(b/2V_a)\ r +C_{n_{\delta_r}}\delta_r+C_{n_{\delta_a}}\delta_a\right]

\end{bmatrix}+

\dfrac{1}{2}\rho S_{\text{prop}}C_{\text{prop}}\begin{bmatrix}0\\0\\0\end{bmatrix}
}$

A continuación se detalla la forma en la cual estas ecuaciones se ven implementadas en este bloque.

![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%2010.png)

## Modelo Aerodinámico - Aerodynamic Coefficients

El modelo aerodinámico utilizado se ha extraido del libro ***********Small Unmnned Aircraft: Theory and Practice - Beard&McLain (***********https://github.com/randybeard/uavbook***********).*********** Los diferentes coeficientes aerodinámicos se pueden subdividir en distintos bloques.

![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%2011.png)

---

### Coeficientes dependientes del AOA $\equiv \alpha$ - Longitudinal Aerodynamics (Cx, Cz, Cm coefficients)

![Donde $F_{\text{lift}} = L,\ F_{\text{drag}} =D$](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%2012.png)

Donde $F_{\text{lift}} = L,\ F_{\text{drag}} =D$

- Estos coeficientes corresponden con la fuerza resultante en el plano $X^bZ^b$ del avión, es decir, el plano longitudinal y con el momento perpendicular a dicho plano.
- Esta fuerza descompuesta en ejes estabilidad $X^s Z^s \rightarrow \textbf{i}^s,\textbf{k}^s$  corresponde con la sustentación (lift) $\textbf{L}=L(-\textbf k^s),\ L>0$ y con la resistencia aerodinámica (drag) $\textbf{D}=D(-\textbf i^s),\ D>0$. Esta misma fuerza proyectada en ejes cuerpo $X^b Z^b \rightarrow \textbf{i}^b,\textbf{k}^b$  se corresponde con los vectores $\textbf{X}=X\ \textbf i^b$y $\textbf{Z}=Z\ \textbf k^b$.
    
    ![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%2013.png)
    
- Lo que finalmente se implementa en el bloque son:
    - Los modelos se definen como - **Stability Axis Aerodynamics block**:
        - $\boxed{C_D(\alpha)= C_{D_p} + C_{D_\alpha}\alpha}$
        - $\boxed{C_L(\alpha)= C_{L_0} + C_{L_\alpha}\alpha}$
        - $\boxed{C_m(\alpha)= C_{m_0} + C_{m_\alpha}\alpha}$
        
        ![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%2014.png)
        
    - Además se tiene que el modelo completo de estos coeficientes es:
        - $\boxed{C_D(\alpha, q, \delta_e)= C_{D}(\alpha) + C_{D_q} \left( \dfrac{c}{2V_a}q\right) + C_{D_{\delta_e}}\delta_e}$
        - $\boxed{C_L(\alpha, q, \delta_e)= C_{L}(\alpha) + C_{L_q} \left( \dfrac{c}{2V_a}q\right) + C_{L_{\delta_e}}\delta_e}$
        - $\boxed{C_m(\alpha, q, \delta_e)= C_{m}(\alpha) + C_{m_q} \left( \dfrac{c}{2V_a}q\right) + C_{m_{\delta_e}}\delta_e}$
    
    Estos coeficientes dependientes de $q$ y de $\delta_e$, se contemplan en el apartado (bloques) siguiente, **********************************************************no se calculan en este bloque**********************************************************.
    
- Para pasar de un eje a otro, se usa una matriz de proyección, definida por el AOA $\equiv \alpha$ - **Stability to Body Axis Matrix Computation block**
    
    $\textbf T_s^b (\alpha) = \begin{bmatrix}
    \cos \alpha&-\sin \alpha\\
    \sin \alpha&\cos \alpha
    \end{bmatrix}$
    
    ![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%2015.png)
    
    De forma que definida la fuerza en ejes cuerpo $\textbf f^{\ b}=  \begin{bmatrix}X\\Z\end{bmatrix}= \begin{bmatrix}C_X\\C_Z\end{bmatrix}\cdot \frac{1}{2} \rho(h)V_a^2 S$ , y en ejes viento  $\textbf f^{\ s}=  \begin{bmatrix}-L\\-D\end{bmatrix}= \begin{bmatrix}-C_L\\-C_D\end{bmatrix}\cdot \frac{1}{2} \rho(h)V_a^2 S$ , se cumple que
    
    $\textbf f^{\ b}=\textbf T_s^b\ \textbf f^{\ s} \Rightarrow\begin{bmatrix}C_X\\C_Z\end{bmatrix}=\textbf T_s^b\begin{bmatrix}-C_L\\-C_D\end{bmatrix}$
    
- Los términos calculados en está sección son:

| $\alpha$ | $C_X(\alpha)= -C_L(\alpha)\cos\alpha +C_D(\alpha)\sin\alpha$ | $C_Z(\alpha)=-C_L(\alpha)\sin\alpha -C_D(\alpha)\cos\alpha$ | $C_m(\alpha)$ |
| --- | --- | --- | --- |

<aside>
⚠️ Los términos mostrados en esta tabla **son los únicos generados en este bloque**. El resto son coeficientes que han de ser multiplicados por su correspondiente variable independiente, para de esta forma generar los términos que componen a los coeficientes adimensionales de las fuerzas y momentos aerodinámicos finales.

</aside>

---

### Cálculo de los términos de los coeficientes adimensionales - alpha-beta Coefficients, Body Rate Damping, Actuator Increments

En este apartado se contemplan los efectos de los coeficientes de la dinámica lateral del avión, es decir la que ocurre fuera del plano $X^bZ^b$, y los efectos de los coeficientes de la dinámica longitudinal $C_{D_q}, C_{L_q}, C_{m_q}, C_{D_{\delta_e}}, C_{L_{\delta_e}}$ y $C_{m_{\delta_e}}$.

- **alpha-beta Coefficients:**  se calculan los términos asociados al slip-side angle $\beta$ y se suman los términos independientes en la fuerza lateral y los momentos en $X^b$ y $Z^b$ (en nuestro caso, nulos al manejar un UAV simétrico). Además, se importan los términos longitudinales y se adicionan en este bloque.
    
    
    | $\alpha$ | $C_{X}(\alpha)$ | $C_{Z}(\alpha)$ | $C_{m}(\alpha)$ |
    | --- | --- | --- | --- |
    | $\beta$ | $C_{Y_\beta}\beta$ | $C_{l_\beta} \beta$ | $C_{n_\beta}\beta$ |
    | Términos independientes | $C_{Y_0}$ | $C_{l_0}$ | $C_{n_0}$ |
    
    ![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%2016.png)
    
- **Body Rate Damping:** se calculan los términos de amortiguamiento de las velocidades angulares $p, q, r$. Estos son
    
    
    | $p$ - p Coefficients block | $C_{Y_p}(b/2V_a)p$ | $C_{l_p}(b/2V_a)p$ | $C_{n_p}(b/2V_a)p $ |
    | --- | --- | --- | --- |
    | $q$ - q Coefficients block | $C_{X_q}(\alpha)(c/2V_a)q$ | $C_{Z_q}(\alpha)(c/2V_a)q$ | $C_{m_q}(c/2V_a)q$ |
    | $r$ - r Coefficients block | $C_{Y_r}(b/2V_a)r$ | $C_{l_r}(b/2V_a)r$ | $C_{n_r}(b/2V_a)r$ |
    
    ![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%2017.png)
    
- **Actuator Increments:** se calculan los términos correspondientes a las deflexiones de las superficies de control $\delta_a,\ \delta_e$ y $\delta_r$ .
    
    
    | $\delta_a$ - Aileron block | $C_{Y_{\delta_a}}\delta_a$ | $ C_{l_{\delta_a}}\delta_a$ | $ C_{n_{\delta_a}}\delta_a$ |
    | --- | --- | --- | --- |
    | $\delta_e$ - Elevator block | $C_{X_{\delta_e}}\delta_e$ | $C_{Z_{\delta_e}}\delta_e$ | $C_{m_{\delta_e}}\delta_e$ |
    | $\delta_r$ - Rudder block | $C_{Y_{\delta_r}}\delta_r$ | $C_{l_{\delta_r}}\delta_r$ | $C_{n_{\delta_r}}\delta_r$ |
    | $\delta_f$ - Flap block | $C_{X_{\delta_f}}\delta_f$ | $C_{Z_{\delta_f}}\delta_f$ | $C_{m_{\delta_f}}\delta_f$ |
    
    ![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%2018.png)
    

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
    
    ![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%2019.png)
    

## Fuerza gravitatoria - Gravity

Dado el vector de aceleración gravitatoria generado en el bloque **Environment Model** se, procede a calcular el peso que actúa sobre la aeronave.

- El vector recibido viene dado en ejes NED, $X^eY^eZ^e$ y pùede escribirse como:

$\mathbf{g}\equiv \mathbf{g}^e =\begin{bmatrix}
0\\
0\\
g
\end{bmatrix}_e$

- Para pasar este vector a ejes cuerpo $X^bY^bZ^b$ (ejes en los cuales están mecanizadas las ecuaciones), se multiplica el vector $\textbf{g}$ mecanizado en ejes NED por la ****************************matriz de cosenos directores**************************** o matriz de rotación entre los ejes cuerpo y los ejes NED, que es generada en el bloque **************6DOF (Quaternion)************** es decir:

$\mathbf{DCM}_e^b = \underbrace{\begin{bmatrix}
\cos\phi&\sin\phi&0 \\
-\sin\phi& \cos\phi& 0\\
 0& 0&1
\end{bmatrix}}_{\text{Roll - }\phi}
\underbrace{\begin{bmatrix}
\cos\theta&0 &-\sin\theta \\
0 &1 &0 \\
\sin\theta& 0&\cos\theta
\end{bmatrix}}_{\text{Pitch- }\theta}
\underbrace{\begin{bmatrix}
\cos\psi &\sin\psi & 0\\
 -\sin\psi&\cos\psi &0 \\
 0&0 &1
\end{bmatrix}}_{\text{Yaw - }\psi},\quad \mathbf{DCM}_b^e = \left(\mathbf{DCM}_e^b\right)^{-1}=\left(\mathbf{DCM}_e^b\right)^{\text{T}}$ 

- Finalmente, se multiplica $\mathbf g^e$ por la masa del avión $\text m$ y por la matriz $\mathbf{DCM}_e^b$

$\mathbf F_g =\text m \mathbf g \equiv \mathbf F_g^b =\text m \mathbf g^b =\text m\cdot \textbf{DCM}_e^b \ \mathbf g^e  = \text mg \begin{bmatrix}
-\sin\theta\\
\cos\theta\sin\phi\\
\cos\theta\cos\phi
\end{bmatrix}$

![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%2020.png)

# Condicionado de las señales de control - Control BUS Creator

Este bloque se encarga de crear el bus de datos `CTRL BUS` , preparando las señales de entrada (PWM en rangos entre 0 y 1, y entre -1 y 1) para ser implementadas en la ecuaciones dinámicas.

- Las señales PWM de los alerones, elevador y rudder (`Ailerons Command (-1,1)`, `Elevator Command (-1,1)`, `Rudder Command (-1,1)`) se multiplican por $\delta_{\text{max}}>0, \textbf[\delta_{\text{max}}\textbf ]\equiv\text{rad}$, de forma que se generan las señales $\delta_a , \delta_e, \delta_r \in (-\delta_{\text{max}},\delta_{\text{max}})$. Este parámetro determina la deflexión máxima de las superficies de control en radianes. Se especifica en el archivo de configuración del modelo `config_model.m` .
- La señal PWM de los flaps puede ir de 0 a 1. Los flaps tienen un bloque de saturación y un rate limiter para emular la variación progresiva del ángulo de los flaps $\delta_f$. Además las posiciones de los flaps están limitadas a 0%, 20%, 60% o  100% de la deflexión máxima posible, es decir $\delta_f\in[0,\ 0.2\times \delta_{\text{max}},\  0.6\times  \delta_{\text{max}},\  \delta_{\text{max}}]$.
- La señal de la palanca de potencia del motor (throttle)  solo es pasada por un saturador para prevenir que esta escape del rango permitido, $\delta_t \in (0,1)$.
- Finalmente se crea el bus de control `CTRL BUS` .

![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%2021.png)

# Modelos atmosférico y gravitatorio - Environment Model

El modelo ambiental utilizado consta de dos submodelos: el modelo gravitatorio (**WGS84 Gravity Model**) y el modelo atmosférico. A su vez, el modelo atmosférico está subdivido en un modelo de atmósfera estática para la determinación de la densidad del aire (**COESA Atmosphere Density Model**) y un modelo de viento (**Wind Models**).

![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%2022.png)

## Modelo Gravitatorio -  **WGS84 Gravity Model**

- El modelo gravitatorio se basa en la distribución uniforme de la masa (equipotencial) del modelo elipsoidal de la tierra WGS84 (World Geodetic System 1984) utilizado para modelar la forma de la Tierra y definir coordenadas de ubicaciones geográficas en su superficie (desarrollado y mantenido por el DoD de los EEUU y la Agencia Nacional de Inteligencia Geoespacial). El tipo de modelo gravitatorio seleccionado es: **********Type of gravity model: WGS84 Taylor Series********** ya que no se necesita exagerada precisión en el modelo para trabajar con drones de corto alcance y baja altitud.
- Se utiliza el bloque **Flat Earth to LLA** para pasar de las coordenadas en ejes tierra/NED $X^eY^eZ^e$ del modelo de tierra plana $[p_n, p_e, p_d]^T$, a coordenadas geodésicas $[\mu, l, h]^T$ donde las variables $\mu, l, h$ se corresponden con la latitud, longitud y altitud de la plataforma.
- La entrada $[\mu, l, h]$ y la salida $[0, 0, g(\mu,l,h)]$ del bloque **WGS84 Gravity Model** son vectores fila (row). Por esta razón se colocan bloques de transposición a la entrada y salida del bloque (ya que todos los vectores en el espacio manejados en el simulador son de dimensión $3\times1$).

## Modelo de Viento - Wind Models

- La velocidad de la aeronave vista desde tierra $\textbf V_g$ (ground velocity) es la suma de la velocidad del avión respecto a la masa de aire circundante $V_a$, también llamada velocidad aerodinámica (air velocity) más la velocidad de dicha más de aire respecto a tierra $\textbf V_w$, es decir, la velocidad del viento (wind velocity).
    
    $\textbf V_g=\textbf V_a+\textbf V_w$
    
- La “airspeed” es el módulo o magnitud de la velocidad aerodinámica: $|\textbf V_a|=V_a=\sqrt{u_a^2+v_a^2+w_a^2}\equiv\text{airspeed}$.
- La velocidad del viento puede descomponerse en una componente determinista y medible $\textbf V_{w_s}$ (steady wind $\partial/\partial t=0$) y una aleatoria $\textbf V_{w_g}$ (random gusts, turbulences):
    
    $\textbf V_g=\textbf V_a+\textbf V_w = \textbf V_a+\textbf V_{w_s}+\underbrace{\textbf V_{w_g}}_{\text{Random}} = 
    \begin{bmatrix}
    u_a\\
    v_a\\
    w_a
    \end{bmatrix}_b
    +
    \begin{bmatrix}
    u_{w}\\
    v_{w}\\
    w_{w}
    \end{bmatrix}_b
    =
    \begin{bmatrix}
    u_a\\
    v_a\\
    w_a
    \end{bmatrix}_b
    +
    \underbrace{
    \begin{bmatrix}
    w_{n_s}\\
    w_{e_s}\\
    w_{d_s}
    \end{bmatrix}_e
    +
    \begin{bmatrix}
    u_{w_g}\\
    v_{w_g}\\
    w_{w_g}
    \end{bmatrix}_b}_{\textbf V_w}$
    
- El steady wind $\textbf V_{w_s}$ puede dividirse en dos componentes: una componente responsable de los vientos cortantes o de cizalladura $\textbf V_{w_{\text{shear}}}$(shear wind)  y una componente de viento uniforme en todo el espacio $\textbf V_{w_{\text{unif}}}$ ($\nabla (\cdot) = [0,0,0]^T$):
    
    $\textbf V_g = \textbf V_a+\textbf V_{w_s}+\textbf V_{w_g} = \textbf V_a+\textbf V_{w_\text{{shear}}}+V_{w_\text{{unif}}}+\textbf V_{w_g}=\begin{bmatrix}
    u_a\\
    v_a\\
    w_a
    \end{bmatrix}_b
    +
    \begin{bmatrix}
    w_{n}^{\text{shear}}\\
    w_{e}^{\text{shear}}\\
    w_{d}^{\text{shear}}
    \end{bmatrix}_e
    +
    \begin{bmatrix}
    w_{n}^{\text{unif}}\\
    w_{e}^{\text{unif}}\\
    w_{d}^{\text{unif}}
    \end{bmatrix}_e
    +
    \begin{bmatrix}
    u_{w_g}\\
    v_{w_g}\\
    w_{w_g}
    \end{bmatrix}_b$
    
- El modelo de Simulink consta de tres submodelos distintos: uno para vientos cortantes o cizalladuras $\textbf V_{w_{\text{shear}}}$ (**Wind Shear Model**), otro para ráfagas de viento turbulentas $\textbf V_{w_g}$ (**Dryden Wind Turbulence Model**) y otro para **ráfagas de viento arbitrarias y controladas** (**Discrete Wind Gust Model**). El viento uniforme es añadido en el archivo de configuración mediante la variable `Vw_unif` .

![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%2023.png)

- Para configurar la ráfaga de viento controlada, abrir el archivo de configuración en la sección  `%% Simulation initial conditions and atmospheric conditions` .
- Para **ACTIVAR** el **Discrete Wind Gust Model** y el **Dryden Wind Turbulence Model**, **hay que entrar en los bloques de Simulink para hacerlo y marcar los checkbox.**

![Untitled](Simulador%20de%20UAVs%20(FWS)-%20V3%20637d736e467345f5893f223daf4063ee/Untitled%2024.png)

- Además, los bloques **Dryden Wind Turbulence Model** y **Wind Shear Model deben configurarse manualmente desde los menús contextuales (masks)** de los bloques de Simulink (se podrán configurar desde el archivo de configuración en versiones posteriores).