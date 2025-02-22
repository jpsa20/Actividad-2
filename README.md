Reporte del Código
1. Objetivo del Código
El presente script de MATLAB se ha diseñado para realizar un análisis cinemático de dos robots de 3 grados de libertad: un robot planar y un robot antropomórfico. Se calculan las matrices de transformación homogénea locales y globales, se obtienen los jacobianos (lineal y angular) mediante métodos diferencial y analítico, y se determinan las velocidades del efector final.

2. Estructura y Variables
Variables simbólicas:
Se utilizan variables simbólicas para los ángulos articulares (con sufijos _pl para el robot planar y _ant para el antropomórfico), así como para las longitudes (o desplazamientos) de cada eslabón.

Configuración de las juntas:
Se especifica que todas las articulaciones son rotacionales (valor 0 en el vector de configuración).

Vectores articulares y sus derivadas:
Se definen los vectores de coordenadas articulares y se calculan sus derivadas temporales para obtener las velocidades articulares.

3. Cálculo de Transformaciones
Transformación Local:
Para el robot planar, cada transformación local se compone de una rotación en z y una traslación a lo largo del eje x.
Para el robot antropomórfico, la primera transformación implica una traslación a lo largo de z y una rotación en un eje distinto (involucrando una componente en y), mientras que las siguientes se asemejan a las del robot planar.

Transformación Global:
Se encadenan las matrices locales para obtener la transformación global de cada eslabón respecto al marco inercial. Esto se realiza mediante un bucle que multiplica secuencialmente las matrices.

4. Cálculo del Jacobiano y Velocidades
Jacobiano Diferencial:
Se calcula tomando las derivadas parciales de la posición final del efector respecto a cada ángulo articular.

Jacobiano Analítico:
Se utiliza el producto cruz y la extracción de la columna correspondiente a la rotación (eje z en el caso planar) para formar el Jacobiano, diferenciando entre juntas rotacionales y prismáticas (aunque en este caso solo hay rotacionales).

Velocidades:
Las velocidades lineal y angular del efector se obtienen multiplicando los jacobianos por el vector de velocidades articulares.

5. Conclusión
El código permite comparar las diferencias en la cinemática entre un robot planar y uno antropomórfico, evidenciando la simplicidad del primero en contraste con la mayor complejidad del segundo en términos de transformación global y cálculo del Jacobiano. La estructura modular y el uso de variables simbólicas facilitan la interpretación y el análisis de cada uno de los componentes del sistema robótico.
