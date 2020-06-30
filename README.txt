la función pronóstico devuelve el vector RUL con la evolución del pronostico de alpha en la columna 1 y el vector de tiempo en la columna 2.
el ultimo valor en el vector de tiempo es el valor de tf para calcular el valor de la RUL=tf-tiempo_actual
La función pronóstico necesita como parametros de entrada [alpha_actual tiempo_actual t_on t_off].
ton >> tiempo en que el motor esta encendido durante el duty cycle. (condiciones de operación)
toff >> tiempo en que el motor esta desactivado durante el duty cycle. (condiciones de operación)
