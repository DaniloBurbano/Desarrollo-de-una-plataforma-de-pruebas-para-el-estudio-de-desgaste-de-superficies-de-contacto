la funci�n pron�stico devuelve el vector RUL con la evoluci�n del pronostico de alpha en la columna 1 y el vector de tiempo en la columna 2.
el ultimo valor en el vector de tiempo es el valor de tf para calcular el valor de la RUL=tf-tiempo_actual
La funci�n pron�stico necesita como parametros de entrada [alpha_actual tiempo_actual t_on t_off].
ton >> tiempo en que el motor esta encendido durante el duty cycle. (condiciones de operaci�n)
toff >> tiempo en que el motor esta desactivado durante el duty cycle. (condiciones de operaci�n)
