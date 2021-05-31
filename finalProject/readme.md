# Readme file to keep track of progress

## TODO

- robot
  - control a bajo nivel.
- environment
  - Rutina para detectar vecinos (grafo de conectividad).
  - Hacer varias iteraciones antes de actualizar el display.
  - Introducir iteraciones
- path_planner
  - ver que algoritmo de pincel queremos
- base: crear clase
  - gestiona que robots hay activos y libera robots una vez cargados.


1. Tener robots en el environment, que el path_planner lanze un objetivo, y los robots hagan la formación.
    - 

# Que hace un time-step del entorno?

1. Loop jefe
   1. El formation_planner ve cual es el siguiente objetivo de la formación.
   2. Loop para alcanzar la formación (n-time skips o detectar cuando se ha alcanzado)
      1. Loop sobre todos los robots activos.
         1. Cada robot pregunta cuales son sus vecinos.
         2. Con esta info, y sabiendo cual es el objetivo de la formación, calculo de velocidad.
         3. Actualizar la posición de los robots.

## DEPENDENCIAS

- Tkinter para sacar la resolución de la pantalla
- Numpy
- cv2 para todo el tema del desplay